/*
 * File:   main_slave.c (Slave PIC - PIC16F873A)
 * Description:
 *   I2C Slave T9 text editor with send/receive messaging on 16x2 LCD.
 *   Line 1: "S: <typed message>"   (send buffer)
 *   Line 2: "R: <received message>" (receive buffer)
 *   '#' queues message for master to read. Master writes arrive via interrupt.
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

#define LCD_RS      PORTAbits.RA0
#define LCD_EN      PORTAbits.RA1

#define KB_PORT     PORTB
#define KB_TRIS     TRISB

#define BUZZER      PORTCbits.RC0

#define SLAVE_ADDR  0x10    // 7-bit address

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

/* ============ Buffers ============ */
static char     sendBuf[MSG_LEN + 1];
static char     recvBuf[MSG_LEN + 1];
static uint8_t  cursorPos;
static uint8_t  sendLen;
static uint8_t  uppercase;

static uint8_t  lastKey;
static uint8_t  tapIndex;
static uint16_t tapTimer;

// I2C TX: message queued for master to read
static volatile char     txBuf[MSG_LEN + 1];
static volatile uint8_t  txLen;       // 0 = no pending message
static volatile uint8_t  txIdx;       // Byte index during master-read
static volatile uint8_t  txSentLen;   // Tracks if length byte was already sent

// I2C RX: incoming message from master
static volatile char     rxTmp[MSG_LEN + 1];
static volatile uint8_t  rxExpected;
static volatile uint8_t  rxIdx;
static volatile uint8_t  rxState;     // 0=waiting for len byte, 1=receiving data

// Flag: ISR sets when a complete message arrives
static volatile uint8_t  newMsgFlag;

/* ============ LCD Functions (PORTA) ============ */

static void lcd_pulse_enable(void) {
    LCD_EN = 1;
    __delay_us(2);
    LCD_EN = 0;
    __delay_us(100);
}

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
    lcd_send_nibble(0x02); __delay_ms(1);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
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
    uint8_t rLen = (uint8_t)strlen((const char*)recvBuf);
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

/* ============ I2C Slave Init ============ */

static void I2C_Slave_Init(void) {
    TRISCbits.TRISC3 = 1;          // SCL
    TRISCbits.TRISC4 = 1;          // SDA
    SSPADD  = SLAVE_ADDR << 1;     // Load 7-bit address
    SSPSTAT = 0x80;                 // Slew rate disabled
    SSPCON  = 0x26;                 // SSP enabled, slave 7-bit mode, CKP released
    SSPCON2 = 0x00;                 // No general call, SEN=0

    // Reset state
    rxState    = 0;
    rxIdx      = 0;
    rxExpected = 0;
    txLen      = 0;
    txIdx      = 0;
    txSentLen  = 0;

    // Enable SSP interrupt
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
}

/* ============ I2C Interrupt Handler ============ */

static void i2c_load_tx_byte(uint8_t value) {
    // Guard SSPBUF writes against write-collision side effects.
    SSPCONbits.WCOL = 0;
    SSPBUF = value;
    if (SSPCONbits.WCOL) {
        SSPCONbits.WCOL = 0;
    }
}

void __interrupt() ISR(void) {
    if (!PIR1bits.SSPIF) return;

    volatile uint8_t dummy;

    // Ignore and clear bus-collision events before continuing normal slave flow.
    if (PIR2bits.BCLIF) {
        PIR2bits.BCLIF = 0;
        PIR1bits.SSPIF = 0;
        SSPCONbits.CKP = 1;
        return;
    }

    // Clear overflow if any
    if (SSPCONbits.SSPOV) {
        SSPCONbits.SSPOV = 0;
        if (SSPSTATbits.BF) dummy = SSPBUF;
        PIR1bits.SSPIF = 0;
        SSPCONbits.CKP = 1;
        return;
    }

    // --- MASTER WRITES to slave (slave is receiving) ---
    // R_nW=0: write operation from master's perspective
    if (!SSPSTATbits.R_nW) {
        if (!SSPSTATbits.D_nA) {
            // Address byte matched ? reset RX state machine
            if (!SSPSTATbits.BF) {
                PIR1bits.SSPIF = 0;
                SSPCONbits.CKP = 1;
                return;
            }
            dummy = SSPBUF;         // Read to clear BF
            rxState    = 0;
            rxIdx      = 0;
            rxExpected = 0;
        } else {
            // Data byte from master
            if (!SSPSTATbits.BF) {
                PIR1bits.SSPIF = 0;
                SSPCONbits.CKP = 1;
                return;
            }
            uint8_t data = SSPBUF;

            if (rxState == 0) {
                // First data byte = message length
                rxExpected = data;
                if (rxExpected > MSG_LEN) rxExpected = MSG_LEN;
                rxIdx   = 0;
                rxState = 1;
            } else {
                // Message content bytes
                if (rxIdx < rxExpected) {
                    rxTmp[rxIdx++] = (char)data;
                }
                if (rxIdx >= rxExpected) {
                    // Complete message ? copy to recvBuf
                    rxTmp[rxIdx] = '\0';
                    memcpy((void*)recvBuf, (const void*)rxTmp, rxIdx + 1);
                    newMsgFlag = 1;
                    rxState = 0;
                }
            }
        }
        SSPCONbits.CKP = 1;  // Release clock

    // --- MASTER READS from slave (slave is transmitting) ---
    // R_nW=1: master wants to read
    } else {
        if (!SSPSTATbits.D_nA) {
            // Address byte matched (read mode) ? prepare first byte (length)
            if (!SSPSTATbits.BF) {
                PIR1bits.SSPIF = 0;
                SSPCONbits.CKP = 1;
                return;
            }
            dummy = SSPBUF;         // Clear BF
            txIdx     = 0;
            txSentLen = 0;
            // Load length byte into buffer
            i2c_load_tx_byte(txLen);
            txSentLen = 1;
        } else {
            // Master requesting next data byte
            if (txLen > 0 && txIdx < txLen) {
                i2c_load_tx_byte((unsigned char)txBuf[txIdx++]);
                // If last byte sent, clear TX buffer
                if (txIdx >= txLen) {
                    txLen = 0;
                }
            } else {
                i2c_load_tx_byte(0x00);  // No data / padding
            }
        }
        SSPCONbits.CKP = 1;  // Release clock to allow master to clock out byte
    }

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
            // Disable interrupt to safely update shared TX buffer
            PIE1bits.SSPIE = 0;
            memcpy((void*)txBuf, sendBuf, sendLen + 1);
            txLen = sendLen;
            txIdx = 0;
            PIE1bits.SSPIE = 1;

            text_clear_send();
            buzzer_beep(150);
        }
        lcd_refresh();
        return;
    }

    if (key == KEY_CONFIRM) {
        multitap_commit();
        lcd_refresh();
        return;
    }

    // T9 multitap
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
    // FIRST: Make PORTA digital
    ADCON1 = 0x07;

    TRISA  = 0x00;
    TRISB  = 0xF0;
    TRISC  = 0x18;      // RC3,RC4 I2C, RC0 buzzer
    PORTA  = 0x00;
    PORTB  = 0x0F;
    PORTC  = 0x00;

    lcd_init();
    I2C_Slave_Init();

    memset(sendBuf, 0, sizeof(sendBuf));
    memset((void*)recvBuf, 0, sizeof(recvBuf));
    memset((void*)txBuf, 0, sizeof(txBuf));
    cursorPos  = 0;
    sendLen    = 0;
    uppercase  = 1;
    lastKey    = KEY_NONE;
    tapIndex   = 0;
    tapTimer   = 0;
    newMsgFlag = 0;

    lcd_refresh();

    uint8_t prevKey = KEY_NONE;

    while (1) {
        // --- Check for received message from ISR ---
        if (newMsgFlag) {
            newMsgFlag = 0;
            buzzer_beep(120);
            lcd_refresh();
        }

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

        __delay_ms(1);
    }
}