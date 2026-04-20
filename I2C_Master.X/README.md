# I2C Master — PIC16F873A

A T9 text editor with bidirectional I2C messaging displayed on a 16×2 LCD.

---

## Hardware

| Signal      | Pin  | Direction |
|-------------|------|-----------|
| LCD RS      | RA0  | Output    |
| LCD EN      | RA1  | Output    |
| LCD D4–D7   | RA2–RA5 | Output |
| KB Rows     | RB0–RB3 | Output |
| KB Cols     | RB4–RB7 | Input (pull-ups) |
| Buzzer      | RC0  | Output    |
| I2C SCL     | RC3  | Open-drain (pull-up required) |
| I2C SDA     | RC4  | Open-drain (pull-up required) |

**Crystal:** 4 MHz XT  
**I2C clock:** 100 kHz (`SSPADD = 9` at 4 MHz)  
**Slave address:** 7-bit `0x10` → write frame `0x20`, read frame `0x21`

---

## Functionality

### LCD layout

```
S: <typed message>      <- Line 1: send buffer (13 chars)
R: <received message>   <- Line 2: last message received from slave
```

### T9 text input

Keys map to character groups identical to a classic phone keypad:

| Key | Characters  |
|-----|-------------|
| 1   | `1 . , ! ?` |
| 2   | `A B C 2`   |
| 3   | `D E F 3`   |
| 4   | `G H I 4`   |
| 5   | `J K L 5`   |
| 6   | `M N O 6`   |
| 7   | `P Q R S 7` |
| 8   | `T U V 8`   |
| 9   | `W X Y Z 9` |
| 0   | `(space) 0` |

Press the same key repeatedly within **800 ms** to cycle through its characters. After the timeout the character is committed automatically.

### Special keys

| Key index | Function               |
|-----------|------------------------|
| 3         | Cursor left            |
| 7         | Cursor right           |
| 11        | Delete character       |
| 12        | Toggle upper/lowercase |
| 14 (`#`)  | Send message via I2C   |
| 15        | Commit multitap        |

### Sending a message

Press `#` to transmit the current send buffer to the slave over I2C. The buffer is cleared and the buzzer beeps (150 ms) to confirm.

### Receiving a message

Every **500 ms** the master polls the slave. If the slave has a pending message, it is copied into the receive buffer, displayed on line 2, and the buzzer beeps (120 ms).

---

## How I2C Works

I2C (Inter-Integrated Circuit) is a two-wire synchronous serial bus. All devices share the same two lines:

- **SCL** — clock, driven by the master.
- **SDA** — data, driven by whoever is transmitting.

Both lines are **open-drain**: devices can only pull the line low; external pull-up resistors pull it high when the line is released.

### Bus conditions

| Condition | Description |
|-----------|-------------|
| **START** | SDA falls while SCL is high — signals the beginning of a transaction. |
| **STOP**  | SDA rises while SCL is high — releases the bus. |
| **Data bit** | SDA is stable while SCL is high; it may change only while SCL is low. |
| **ACK**   | After each byte the receiver pulls SDA low for one clock cycle to acknowledge. |
| **NACK**  | SDA is left high (released) by the receiver — signals an error or end of transfer. |

### Addressing

Each byte transmitted after START begins with the **7-bit slave address** followed by a **direction bit**:

```
[ A6 A5 A4 A3 A2 A1 A0 | R/W ]
```

- `R/W = 0` → master writes to slave.  
- `R/W = 1` → master reads from slave.

This project uses slave address `0x10`:  
- Write frame: `0x20` (`0001 0000 0`)  
- Read frame:  `0x21` (`0001 0000 1`)

### PIC16F873A SSP module (MSSP in I2C Master mode)

The PIC's SSP peripheral handles the low-level bit-banging automatically. The firmware controls it through three registers:

| Register | Role |
|----------|------|
| `SSPCON`  | Enable SSP, select I2C Master mode (`0x28`) |
| `SSPCON2` | Initiate START (`SEN`), STOP (`PEN`), receive (`RCEN`), ACK/NACK (`ACKEN`, `ACKDT`) |
| `SSPSTAT` | Bus-busy flag (`R_W`), buffer-full flag (`BF`), slew-rate control |
| `SSPADD`  | Baud-rate divider: `Fosc / (4 × Fscl) − 1` |
| `SSPBUF`  | Transmit/receive data register |
| `SSPIF`   | Interrupt flag set by hardware when each bus event completes |

#### SSPIF — the synchronisation flag

`PIR1bits.SSPIF` is set by hardware when the SSP finishes any bus operation (START, STOP, byte transmit, byte receive, or ACK/NACK). The correct sequence for every operation is:

1. Clear SSPIF (`PIR1bits.SSPIF = 0`).
2. Trigger the operation (set `SEN`, write `SSPBUF`, set `RCEN`, etc.).
3. Wait for SSPIF to become `1` (`while (!PIR1bits.SSPIF)`).
4. Clear SSPIF again before the next operation.

Polling individual control bits such as `SEN` or `ACKEN` directly is unreliable because hardware clears them before the bus event is fully settled. Using SSPIF is the method recommended by the PIC16F873A datasheet.

### Write transaction (master → slave)

```
Master: START | ADDR+W | DATA[0] | DATA[1] | ... | STOP
Slave:               ACK       ACK       ACK
```

Protocol used by this project when sending a message:

```
START → SLAVE_ADDR_W → len → data[0] → data[1] → ... → STOP
```

### Read transaction (master ← slave)

```
Master: START | ADDR+R |     | RCV[0] | ACK | RCV[1] | ACK | RCV[n] | NACK | STOP
Slave:               ACK | DATA[0] |     | DATA[1] |     | DATA[n] |
```

Protocol used by this project when polling the slave:

```
START → SLAVE_ADDR_R → read len byte
  if len == 0: NACK → STOP   (no message)
  else:        ACK  → read data[0..n-1] with ACK, data[n] with NACK → STOP
```

NACK on the last byte tells the slave that the master has finished reading and will release the bus.

---

## I2C Function Reference

| Function | Description |
|----------|-------------|
| `I2C_Init_Master()` | Configure SSP for 100 kHz I2C Master mode |
| `I2C_Wait()` | Block while the bus or any control bit is still busy |
| `I2C_Start()` | Issue a START condition, wait for SSPIF |
| `I2C_Stop()` | Issue a STOP condition, wait for SSPIF |
| `I2C_Write(data)` | Transmit one byte, return `0` on ACK / `1` on NACK |
| `I2C_Read()` | Receive one byte from slave |
| `I2C_Ack()` | Send ACK after a received byte |
| `I2C_Nack()` | Send NACK after the last received byte |
| `I2C_SendMessage(msg, len)` | High-level: send a text message to the slave |
| `I2C_PollMessage(buf, maxLen)` | High-level: poll slave for an incoming message |
