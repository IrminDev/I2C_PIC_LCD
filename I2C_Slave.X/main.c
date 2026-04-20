/*
 * File:   main_slave.c (Slave PIC - PIC16F873A)
 * Description:
 *   I2C Slave T9 text editor with send/receive messaging on 16x2 LCD.
 *   Line 1: "S: <typed message>"   (send buffer)
 *   Line 2: "R: <received message>" (receive buffer)
 *   '#' queues message for master to read. Master writes arrive via interrupt.
 *
 * Protocol (matching master):
 *   Master write ? slave: len + data[0..len-1]
 *   Master read  ? slave: len + data[0..len-1]
 *
 * Wiring: same as master (LCD on PORTA, KB on PORTB, I2C on RC3/RC4, buzzer RC0)
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

#define LCD_RS      PORTAbits.RA0
#define LCD_EN      PORTAbits.RA1

#define KB_PORT     PORTB

#define BUZZER      PORTCbits.RC0

#define SLAVE_ADDR  0x10

#define LCD_COLS            16
#define MSG_LEN             13
#define DEBOUNCE_MS         50
#define MULTITAP_TIMEOUT_MS 800

#define KEY_CURSOR_LEFT     3
#define KEY_CURSOR_RIGHT    7
#define KEY_DELETE           11
#define KEY_TOGGLE_CASE     12
#define KEY_SEND            14
#define KEY_CONFIRM         15
#define KEY_NONE            0xFF

static const char *keyChars[16] = {
    "1.,!?", "ABC2", "DEF3", NULL,
    "GHI4",  "JKL5", "MNO6", NULL,
    "PQRS7", "TUV8", "WXYZ9", NULL,
    NULL,     " 0",   NULL,    NULL
};

static char     sendBuf[MSG_LEN + 1];
static char     recvBuf[MSG_LEN + 1];
static uint8_t  cursorPos;
static uint8_t  sendLen;
static uint8_t  uppercase;

static uint8_t  lastKey;
static uint8_t  tapIndex;
static uint16_t tapTimer;

// TX: pending message for master to read
static volatile char     txBuf[MSG_LEN + 1];
static volatile uint8_t  txLen;
static volatile uint8_t  txIdx;
static volatile uint8_t  txConsumed;  // Flag: master finished reading

// RX: message being received from master
static volatile char     rxTmp[MSG_LEN + 1];
static volatile uint8_t  rxExpected;
static volatile uint8_t  rxIdx;
static volatile uint8_t  rxState;     // 0=expect len, 1=expect data

static volatile uint8_t  newMsgFlag;

/* ============ LCD (PORTA) ============ */

static void lcd_pulse_enable(void) {
    LCD_EN = 1;
    __delay_us(4);
    LCD_EN = 0;
    __delay_us(100);
}

static void lcd_send_nibble(uint8_t nibble) {
    uint8_t tmp = PORTA & 0x03;
    PORTA = tmp | ((nibble & 0x0F) << 2);
    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t rs, uint8_t data) {
    LCD_RS = rs;
    __delay_us(5);
    lcd_send_nibble(data >> 4);
    lcd_send_nibble(data & 0x0F);
    __delay_us(120);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(0, cmd);
    if (cmd <= 0x03) __delay_ms(3);
}

static void lcd_char(char c) {
    lcd_send_byte(1, (uint8_t)c);
}

static void lcd_print(const char *str) {
    while (*str) lcd_char(*str++);
}

static void lcd_init(void) {
    PORTA = 0x00;
    __delay_ms(40);

    LCD_RS = 0;
    lcd_send_nibble(0x03); __delay_ms(5);
    lcd_send_nibble(0x03); __delay_ms(1);
    lcd_send_nibble(0x03); __delay_ms(1);
    lcd_send_nibble(0x02); __delay_ms(2);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    __delay_ms(3);
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
    uint8_t rLen = (uint8_t)strlen((const char *)recvBuf);
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
        PORTB = (~(1 << row)) & 0x0F;
        __delay_us(10);
        uint8_t cols = (PORTB >> 4) & 0x0F;
        if (cols != 0x0F) {
            for (col = 0; col < 4; col++) {
                if (!(cols & (1 << col)))
                    return (uint8_t)(row * 4 + col);
            }
        }
    }
    PORTB = 0x0F;
    return KEY_NONE;
}

/* ============ I2C Slave Init ============ */

static void I2C_Slave_Init(void) {
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    SSPADD  = SLAVE_ADDR << 1;   // 0x10 << 1 = 0x20
    SSPSTAT = 0x80;               // Slew rate off
    SSPCON  = 0x36;               // SSP on, slave 7-bit, CKP=1
    SSPCON2 = 0x01;               // SEN=1 (clock stretching)

    rxState    = 0;
    rxIdx      = 0;
    rxExpected = 0;
    txLen      = 0;
    txIdx      = 0;
    txConsumed = 0;

    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
}

/* ============ I2C ISR ============ */

void __interrupt() ISR(void) {
    if (!PIR1bits.SSPIF) return;

    unsigned char dummy;

    // Clear overflow
    if (SSPCONbits.SSPOV) {
        SSPCONbits.SSPOV = 0;
        dummy = SSPBUF;
    }

    /* ---- Master writes to slave (slave receives) ---- */
    if (!SSPSTATbits.R_nW) {

        if (!SSPSTATbits.D_nA) {
            // Address match ? clear BF, reset RX state
            dummy = SSPBUF;
            rxState    = 0;
            rxIdx      = 0;
            rxExpected = 0;
        } else {
            // Data byte from master
            unsigned char data = SSPBUF;

            if (rxState == 0) {
                // First byte = message length
                rxExpected = data;
                if (rxExpected > MSG_LEN)
                    rxExpected = MSG_LEN;
                rxIdx = 0;
                if (rxExpected > 0)
                    rxState = 1;
            } else {
                // Content bytes
                if (rxIdx < rxExpected)
                    rxTmp[rxIdx++] = (char)data;

                if (rxIdx >= rxExpected) {
                    // Complete ? copy to recvBuf byte by byte
                    rxTmp[rxIdx] = '\0';
                    for (uint8_t k = 0; k <= rxIdx; k++)
                        recvBuf[k] = rxTmp[k];
                    newMsgFlag = 1;
                    rxState = 0;
                }
            }
        }
    }

    /* ---- Master reads from slave (slave transmits) ---- */
    else {

        if (!SSPSTATbits.D_nA) {
            // Address match (read) ? send length as first byte
            dummy = SSPBUF;
            txIdx = 0;
            SSPBUF = txLen;
        } else {
            // Master wants next data byte
            if (txLen > 0 && txIdx < txLen) {
                SSPBUF = (unsigned char)txBuf[txIdx++];
                // Mark consumed when last byte loaded
                if (txIdx >= txLen)
                    txConsumed = 1;
            } else {
                SSPBUF = 0x00;
            }
        }
    }

    // Release clock ? MUST be done for every interrupt
    SSPCONbits.CKP = 1;

    PIR1bits.SSPIF = 0;
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

    // '#' = Queue message for master to read, clear send line
    if (key == KEY_SEND) {
        multitap_commit();
        if (sendLen > 0) {
            // Atomic update of TX buffer
            INTCONbits.GIE = 0;
            for (uint8_t k = 0; k <= sendLen; k++)
                txBuf[k] = sendBuf[k];
            txLen = sendLen;
            txIdx = 0;
            txConsumed = 0;
            INTCONbits.GIE = 1;

            text_clear_send();
            buzzer_beep(100);
            __delay_ms(50);
            buzzer_beep(100);
        }
        lcd_refresh();
        return;
    }

    if (key == KEY_CONFIRM) {
        multitap_commit();
        lcd_refresh();
        return;
    }

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
    ADCON1 = 0x07;

    TRISA  = 0x00;
    TRISB  = 0xF0;
    TRISC  = 0x18;
    PORTA  = 0x00;
    PORTB  = 0x0F;
    PORTC  = 0x00;

    lcd_init();
    I2C_Slave_Init();

    memset(sendBuf, 0, sizeof(sendBuf));
    memset((void *)recvBuf, 0, sizeof(recvBuf));
    memset((void *)txBuf, 0, sizeof(txBuf));
    memset((void *)rxTmp, 0, sizeof(rxTmp));
    cursorPos  = 0;
    sendLen    = 0;
    uppercase  = 1;
    lastKey    = KEY_NONE;
    tapIndex   = 0;
    tapTimer   = 0;
    newMsgFlag = 0;
    txConsumed = 0;

    lcd_refresh();

    lcd_set_cursor(0, 1);
    lcd_print("R:              ");
    __delay_ms(1000);
    lcd_refresh();

    uint8_t prevKey = KEY_NONE;

    while (1) {
        // Safely clear TX buffer after master consumed it
        if (txConsumed) {
            INTCONbits.GIE = 0;
            txLen = 0;
            txConsumed = 0;
            INTCONbits.GIE = 1;
        }

        // New message received from master
        if (newMsgFlag) {
            newMsgFlag = 0;
            buzzer_beep(80);
            __delay_ms(30);
            buzzer_beep(80);
            lcd_refresh();
        }

        // Keyboard
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

        // Multitap timeout
        if (tapTimer > 0) {
            tapTimer--;
            if (tapTimer == 0) multitap_commit();
        }

        __delay_ms(1);
    }
}