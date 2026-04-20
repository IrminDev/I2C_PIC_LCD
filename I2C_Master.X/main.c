/*
 * File:   main.c (Master PIC - PIC16F873A)
 * Description:
 *   I2C Master T9 text editor with send/receive messaging on 16x2 LCD.
 *   Line 1: "S: <typed message>"   (send buffer)
 *   Line 2: "R: <received message>" (receive buffer)
 *   '#' sends message via I2C to slave, clears send buffer.
 *   Master periodically polls slave for incoming messages.
 *
 * Wiring (PIC16F873A):
 *   LCD D4-D7  -> RA2-RA5  (RA4 needs external pull-up, open-drain)
 *   LCD RS     -> RA0
 *   LCD EN     -> RA1
 *   KB Rows    -> RB0-RB3 (output)
 *   KB Cols    -> RB4-RB7 (input, external pull-ups)
 *   I2C SCL    -> RC3
 *   I2C SDA    -> RC4
 *   Buzzer     -> RC0
 */

#pragma config FOSC  = XT
#pragma config WDTE  = OFF
#pragma config PWRTE = ON
#pragma config CP    = OFF
#pragma config BOREN = ON
#pragma config LVP   = OFF
#pragma config CPD   = OFF
#pragma config WRT   = OFF

#include <xc.h>
#include <stdint.h>
#include <string.h>

#define _XTAL_FREQ 4000000UL

/* ============ Pin Definitions ============ */
#define LCD_RS      PORTAbits.RA0
#define LCD_EN      PORTAbits.RA1

#define KB_PORT     PORTB
#define KB_TRIS     TRISB

#define BUZZER      PORTCbits.RC0

// I2C slave address: 7-bit 0x10 ? write=0x20, read=0x21
#define SLAVE_ADDR_W  0x20
#define SLAVE_ADDR_R  0x21

/* ============ Constants ============ */
#define LCD_COLS            16
#define MSG_LEN             13  // Usable chars per line (16 - 3 for "S: ")
#define DEBOUNCE_MS         50
#define MULTITAP_TIMEOUT_MS 800
#define POLL_INTERVAL       500

#define KEY_CURSOR_LEFT     3
#define KEY_CURSOR_RIGHT    7
#define KEY_DELETE           11
#define KEY_TOGGLE_CASE     12
#define KEY_SEND            14  // '#' ? send message
#define KEY_CONFIRM         15  // 'D' ? commit multitap
#define KEY_NONE            0xFF

/* ============ T9 Keymap ============ */
static const char *keyChars[16] = {
    "1.,!?", "ABC2", "DEF3", NULL,
    "GHI4",  "JKL5", "MNO6", NULL,
    "PQRS7", "TUV8", "WXYZ9", NULL,
    NULL,     " 0",   NULL,    NULL
};

/* ============ Global State ============ */
static char     sendBuf[MSG_LEN + 1];
static char     recvBuf[MSG_LEN + 1];
static uint8_t  cursorPos;
static uint8_t  sendLen;
static uint8_t  uppercase;

static uint8_t  lastKey;
static uint8_t  tapIndex;
static uint16_t tapTimer;
static uint16_t pollTimer;

/* ============ LCD Functions (PORTA) ============ */

static void lcd_pulse_enable(void) {
    LCD_EN = 1;
    __delay_us(2);
    LCD_EN = 0;
    __delay_us(100);
}

// Data nibble on RA2-RA5
static void lcd_send_nibble(uint8_t nibble) {
    PORTA = (PORTA & 0x03) | ((nibble & 0x0F) << 2);
    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t rs, uint8_t data) {
    LCD_RS = rs;
    __delay_us(5);
    lcd_send_nibble(data >> 4);
    lcd_send_nibble(data & 0x0F);
    __delay_us(100);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(0, cmd);
    if (cmd <= 0x03) __delay_ms(2);
}

static void lcd_char(char c) {
    lcd_send_byte(1, (uint8_t)c);
}

static void lcd_init(void) {
    PORTA = 0x00;
    __delay_ms(30);

    LCD_RS = 0;
    lcd_send_nibble(0x03); __delay_ms(5);
    lcd_send_nibble(0x03); __delay_ms(1);
    lcd_send_nibble(0x03); __delay_ms(1);
    lcd_send_nibble(0x02); __delay_ms(1);  // 4-bit mode

    lcd_cmd(0x28);  // 2 lines
    lcd_cmd(0x0C);  // Display on, cursor off
    lcd_cmd(0x06);  // Auto-increment
    lcd_cmd(0x01);  // Clear
    __delay_ms(2);
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    lcd_cmd((row == 0) ? (0x80 + col) : (0xC0 + col));
}

static void lcd_refresh(void) {
    uint8_t i;

    lcd_set_cursor(0, 0);
    lcd_char('S'); lcd_char(':'); lcd_char(' ');
    for (i = 0; i < MSG_LEN; i++)
        lcd_char((i < sendLen) ? sendBuf[i] : ' ');

    lcd_set_cursor(0, 1);
    lcd_char('R'); lcd_char(':'); lcd_char(' ');
    uint8_t rLen = (uint8_t)strlen(recvBuf);
    for (i = 0; i < MSG_LEN; i++)
        lcd_char((i < rLen) ? recvBuf[i] : ' ');

    lcd_set_cursor(3 + cursorPos, 0);
    lcd_cmd(0x0F);
}

/* ============ Buzzer ============ */

static void buzzer_beep(uint16_t duration_ms) {
    BUZZER = 1;
    while (duration_ms--) __delay_ms(1);
    BUZZER = 0;
}

/* ============ Keyboard (PORTB) ============ */

static uint8_t kb_scan(void) {
    uint8_t row, col;
    for (row = 0; row < 4; row++) {
        TRISB = 0xF0;
        KB_PORT = (~(1 << row)) & 0x0F;
        __delay_us(10);
        uint8_t cols = (KB_PORT >> 4) & 0x0F;
        if (cols != 0x0F) {
            for (col = 0; col < 4; col++) {
                if (!(cols & (1 << col)))
                    return (uint8_t)(row * 4 + col);
            }
        }
    }
    KB_PORT = 0x0F;
    return KEY_NONE;
}

/* ============ I2C Master Functions ============ */

static void I2C_Init_Master(void) {
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    SSPSTAT = 0x80;         // Slew rate disabled
    SSPCON  = 0x28;         // I2C Master, SSP enabled
    SSPADD  = 9;            // 100kHz @ 4MHz
    PIR1bits.SSPIF = 0;
}

static void I2C_Wait(void) {
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

static void I2C_Start(void) {
    I2C_Wait();
    PIR1bits.SSPIF = 0;   
    SSPCON2bits.SEN = 1;
    while(!PIR1bits.SSPIF);
}

static void I2C_Stop(void) {
    I2C_Wait();
    PIR1bits.SSPIF = 0;     // Limpiar antes de iniciar
    SSPCON2bits.PEN = 1;
    while(!PIR1bits.SSPIF); 
}

static unsigned char I2C_Write(unsigned char data) {
    PIR1bits.SSPIF = 0; 
    SSPBUF = data;         
    while(!PIR1bits.SSPIF);
    return SSPCON2bits.ACKSTAT;
}

static unsigned char I2C_Read(void) {
    I2C_Wait();
    PIR1bits.SSPIF = 0;  
    SSPCON2bits.RCEN = 1;    
    while(!PIR1bits.SSPIF);
    return SSPBUF;
}

static void I2C_Ack(void) {
    I2C_Wait();
    SSPCON2bits.ACKDT = 0;
    PIR1bits.SSPIF = 0;
    SSPCON2bits.ACKEN = 1;
    while(!PIR1bits.SSPIF);
}

static void I2C_Nack(void) {
    I2C_Wait();
    SSPCON2bits.ACKDT = 1;  // NACK
    SSPCON2bits.ACKEN = 1;
    while (SSPCON2bits.ACKEN);
}

/* ============ I2C Send Message to Slave ============ */
// Protocol: START ? addr+W ? len ? data[0..len-1] ? STOP
static void I2C_SendMessage(const char *msg, uint8_t len) {
    I2C_Start();
    if (I2C_Write(SLAVE_ADDR_W)) {  // NACK = slave not present
        I2C_Stop();
        return;
    }
    I2C_Write(len);
    for (uint8_t i = 0; i < len; i++)
        I2C_Write((unsigned char)msg[i]);
    I2C_Stop();
}

/* ============ I2C Poll Slave for Message ============ */
// Protocol: START ? addr+R ? read len ? [read data bytes] ? STOP
// Returns bytes received (0 = no message)
static uint8_t I2C_PollMessage(char *buf, uint8_t maxLen) {
    I2C_Start();
    if (I2C_Write(SLAVE_ADDR_R)) {  // NACK = slave not responding
        I2C_Stop();
        return 0;
    }

    // Read message length from slave
    uint8_t len = I2C_Read();

    if (len == 0 || len > maxLen) {
        // No message or invalid length ? NACK and stop
        I2C_Nack();
        I2C_Stop();
        return 0;
    }

    // Valid message: ACK the length byte, then read data
    I2C_Ack();

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = (char)I2C_Read();
        if (buf[i] == (char)0xFF) {
            I2C_Nack();
            I2C_Stop();
            buf[i] = '\0';
            return i;
        }
        if (i < len - 1)
            I2C_Ack();
        else
            I2C_Nack();
    }
    buf[len] = '\0';
    I2C_Stop();
    return len;
}

/* ============ Text Editing ============ */

static void text_insert_char(char c) {
    if (sendLen >= MSG_LEN) return;
    for (int8_t i = (int8_t)sendLen; i > (int8_t)cursorPos; i--)
        sendBuf[i] = sendBuf[i - 1];
    sendBuf[cursorPos] = c;
    sendLen++;
    cursorPos++;
    sendBuf[sendLen] = '\0';
}

static void text_replace_prev_char(char c) {
    if (cursorPos == 0) return;
    sendBuf[cursorPos - 1] = c;
}

static void text_delete(void) {
    if (cursorPos == 0 || sendLen == 0) return;
    for (uint8_t i = cursorPos - 1; i < sendLen - 1; i++)
        sendBuf[i] = sendBuf[i + 1];
    sendLen--;
    cursorPos--;
    sendBuf[sendLen] = '\0';
}

static void text_clear_send(void) {
    memset(sendBuf, 0, sizeof(sendBuf));
    sendLen   = 0;
    cursorPos = 0;
}

/* ============ Multitap ============ */

static void multitap_commit(void) {
    lastKey  = KEY_NONE;
    tapIndex = 0;
    tapTimer = 0;
}

static char apply_case(char c) {
    if (c >= 'A' && c <= 'Z')
        return uppercase ? c : (char)(c + 32);
    return c;
}

/* ============ Key Processing ============ */

static void process_key(uint8_t key) {
    if (key == KEY_CURSOR_LEFT) {
        multitap_commit();
        if (cursorPos > 0) cursorPos--;
        lcd_refresh();
        return;
    }
    if (key == KEY_CURSOR_RIGHT) {
        multitap_commit();
        if (cursorPos < sendLen) cursorPos++;
        lcd_refresh();
        return;
    }
    if (key == KEY_DELETE) {
        multitap_commit();
        text_delete();
        lcd_refresh();
        return;
    }
    if (key == KEY_TOGGLE_CASE) {
        multitap_commit();
        uppercase = !uppercase;
        lcd_refresh();
        return;
    }

    // '#' = Send message via I2C, then clear send buffer
    if (key == KEY_SEND) {
        multitap_commit();
        if (sendLen > 0) {
            I2C_SendMessage(sendBuf, sendLen);
            text_clear_send();
            buzzer_beep(150);
        }
        lcd_refresh();
        return;
    }

    // 'D' = Commit multitap
    if (key == KEY_CONFIRM) {
        multitap_commit();
        lcd_refresh();
        return;
    }

    // T9 multitap input
    const char *chars = keyChars[key];
    if (chars == NULL) return;
    uint8_t len = (uint8_t)strlen(chars);

    if (key == lastKey && tapTimer > 0) {
        tapIndex = (tapIndex + 1) % len;
        text_replace_prev_char(apply_case(chars[tapIndex]));
        tapTimer = MULTITAP_TIMEOUT_MS;
    } else {
        multitap_commit();
        tapIndex = 0;
        text_insert_char(apply_case(chars[0]));
        lastKey  = key;
        tapTimer = MULTITAP_TIMEOUT_MS;
    }
    lcd_refresh();
}

/* ============ Main ============ */

void main(void) {
    // FIRST: Make PORTA digital before any LCD access
    ADCON1 = 0x07;

    TRISA  = 0x00;
    TRISB  = 0xF0;
    TRISC  = 0x18;      // RC3,RC4 for I2C, RC0 buzzer output
    PORTA  = 0x00;
    PORTB  = 0x0F;
    PORTC  = 0x00;

    lcd_init();
    I2C_Init_Master();

    memset(sendBuf, 0, sizeof(sendBuf));
    memset(recvBuf, 0, sizeof(recvBuf));
    cursorPos = 0;
    sendLen   = 0;
    uppercase = 1;
    lastKey   = KEY_NONE;
    tapIndex  = 0;
    tapTimer  = 0;
    pollTimer = 0;

    lcd_refresh();

    uint8_t prevKey = KEY_NONE;

    while (1) {
        // --- Keyboard ---
        uint8_t key = kb_scan();

        if (key != KEY_NONE && key != prevKey) {
            __delay_ms(DEBOUNCE_MS);
            if (kb_scan() == key) {
                buzzer_beep(80);
                process_key(key);
                prevKey = key;
            }
        } else if (key == KEY_NONE) {
            prevKey = KEY_NONE;
        }

        // --- Multitap timeout ---
        if (tapTimer > 0) {
            tapTimer--;
            if (tapTimer == 0) multitap_commit();
        }

        // --- Poll slave for incoming message ---
        pollTimer++;
        if (pollTimer >= POLL_INTERVAL) {
            pollTimer = 0;
            char tmpBuf[MSG_LEN + 1];
            uint8_t rxLen = I2C_PollMessage(tmpBuf, MSG_LEN);
            if (rxLen > 0) {
                strncpy(recvBuf, tmpBuf, MSG_LEN);
                recvBuf[MSG_LEN] = '\0';
                buzzer_beep(120);
                lcd_refresh();
            }
        }

        __delay_ms(1);
    }
}