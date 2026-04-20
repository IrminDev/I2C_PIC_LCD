# I2C_Slave.X

PIC16F873A firmware for an I2C slave node with a 16x2 LCD UI, keypad text entry (T9-style multitap), buzzer feedback, and bidirectional messaging with an I2C master.

## What this project does

- Runs on PIC16F873A at 4 MHz.
- Exposes the MCU as an I2C slave at 7-bit address `0x10`.
- Lets the user type a message on a 4x4 keypad.
- Shows local outgoing text on LCD line 1 (`S:`) and last received text on LCD line 2 (`R:`).
- Sends the typed message to the master when `#` is pressed (queues it for next master read).
- Receives messages written by the master and updates the receive line.

Main implementation is in `main.c`.

## Hardware mapping

- LCD D4-D7: RA2-RA5
- LCD RS: RA0
- LCD EN: RA1
- Keypad rows: RB0-RB3 (outputs)
- Keypad cols: RB4-RB7 (inputs, pull-ups required)
- I2C SCL: RC3
- I2C SDA: RC4
- Buzzer: RC0

## UI behavior

- Line 1 (`S:`) shows editable outgoing text (max 13 chars).
- Line 2 (`R:`) shows most recently received text (max 13 chars).
- Supported edit actions: cursor left/right, delete, case toggle, confirm.
- Multitap timeout commits a character after inactivity.

## I2C communication design

This firmware uses the MSSP peripheral in **I2C slave mode** with interrupt-driven handling.

### Protocol format

Both directions use a length-prefixed payload:

1. First byte: payload length (`0..13`)
2. Next bytes: message data

If a sender provides a length larger than 13, the firmware clamps it to 13.

### Slave address

- 7-bit slave address: `0x10`
- Register load: `SSPADD = SLAVE_ADDR << 1`

### Master write to slave (master TX, slave RX)

When the master writes to this slave:

1. ISR receives address phase and resets RX state.
2. First data byte is interpreted as message length.
3. Following bytes are copied into a temporary RX buffer.
4. When expected length is reached, the message is null-terminated and copied to `recvBuf`.
5. `newMsgFlag` is set so the main loop can refresh LCD and beep.

### Master read from slave (master RX, slave TX)

When the master reads from this slave:

1. On read address phase, slave loads first TX byte = queued message length.
2. On subsequent clocks, slave sends queued message bytes sequentially.
3. After last byte is sent, TX length is cleared (message considered consumed).
4. If no pending message exists, slave returns `0x00`.

### SSPIF validation and error guards

The interrupt handler validates MSSP event state before consuming bytes:

- ISR exits immediately unless `PIR1bits.SSPIF` is set.
- Bus collision (`PIR2bits.BCLIF`) is cleared and transaction is safely aborted.
- Overflow (`SSPCONbits.SSPOV`) is cleared, pending buffer is drained if needed, and ISR exits.
- Buffer-full (`SSPSTATbits.BF`) is checked before each `SSPBUF` read.
- TX writes use guarded helper logic to detect and clear write collision (`SSPCONbits.WCOL`).
- Clock is released with `CKP = 1` on all handled paths so the bus does not stall.

These checks reduce malformed transfers and ISR-side state corruption under bus timing edge cases.

## Build

This is an MPLAB X generated XC8 project.

From project root:

```bash
make build CONF=default
```

Expected production hex output:

`dist/default/production/I2C_Slave.X.production.hex`

## Source structure

- `main.c`: LCD, keypad, buzzer, I2C slave ISR, text editing logic
- `nbproject/`: MPLAB X project metadata and generated make includes
- `dist/`: build artifacts
- `build/`: intermediate objects/dependencies

