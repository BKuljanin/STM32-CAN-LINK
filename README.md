# STM32 CAN Bus Motor Control Link
### NUCLEO-F446RE (Flight Controller) + STM32F103C6 Blue Pill (Motor Controller)

**Note:**
This project implements a two node CAN bus communication link between STM32 Nucleo-F446RE acting as a flight controller and STM32F103C6 (Bluepill) acting as a motor controller.
The flight controller sends motor commands at **1 kHz**, and the motor controller returns status feedback at **100 Hz**.

---

## This project demonstrates

- Two node **CAN bus** communication using **bxCAN** peripheral on STM32
- Defined CAN message protocol
- Periodic transmission from **SysTick** interrupt (1 kHz command, 100 Hz feedback)
- Hardware acceptance filtering (**ID mask mode**)
- Interrupt-driven reception via **RX FIFO** callbacks

---

## CAN Protocol

Two message types are defined on the bus:

### Motor Command (Flight Controller → Motor Controller)

| Byte | Signal          | Type     | Scale       | Unit | Range         |
|-----:|-----------------|----------|-------------|------|---------------|
| 0-1  | Speed Setpoint  | uint16   | 0.1 RPM/bit | RPM  | 0–6553.5      |
| 2    | Enable          | uint8    | —           | —    | 0=off, 1=on   |
| 3    | Direction       | uint8    | —           | —    | 0=CW, 1=CCW   |
| 4-7  | Reserved        | —        | —           | —    | —             |

- **CAN ID:** 0x446
- **DLC:** 8 (number of bytes in a message)
- **Transmit rate:** 1 kHz (every SysTick). This is done for testing, in a real application motor command will be sent as soon as flight controller calculates new motor speed setpoint.

### Motor Status (Motor Controller → Flight Controller)

| Byte | Signal          | Type     | Scale        | Unit | Range         |
|-----:|-----------------|----------|--------------|------|---------------|
| 0-1  | Actual Speed    | uint16   | 0.1 RPM/bit  | RPM  | 0–6553.5      |
| 2-3  | DC Bus Voltage  | uint16   | 0.01 V/bit   | V    | 0–655.35      |
| 4-5  | Phase Current   | uint16   | 0.01 A/bit   | A    | 0–655.35      |
| 6    | Status Flags    | uint8    | —            | —    | —             |
| 7    | Error Code      | uint8    | —            | —    | —             |

- **CAN ID:** 0x103
- **DLC:** 8 (number of bytes in a message)
- **Transmit rate:** 100 Hz (every 10 ms via SysTick counter)

All multi-byte signals use **big endian** byte order (MSB first).

---

## CAN Bus Configuration

| Parameter         | Nucleo (F446RE)     | Blue Pill (F103C6)  |
|------------------:|---------------------|---------------------|
| APB1 Clock        | 45 MHz              | 36 MHz              |
| Prescaler         | 18                  | 18                  |
| Time Segment 1    | 2 TQ                | 2 TQ                |
| Time Segment 2    | 2 TQ                | 1 TQ                |
| Baud Rate         | 500 kbps            | 500 kbps            |
| TX Pin            | PA12 (AF9)          | PA12 (AF Push-pull) |
| RX Pin            | PA11 (AF9)          | PA11 (Input)        |
| RX FIFO           | FIFO0               | FIFO1               |
| Filter Bank       | 18                  | 10                  |
| Filter Mode       | ID Mask, 32-bit     | ID Mask, 32-bit     |
| Accepted ID       | 0x103               | 0x446               |
| Filter Mask       | 0x7FF (all 11 bits) | 0x7FF (all 11 bits) |

---

## High-Level Flow

### Nucleo (Flight Controller)

1. System clock configured to **180 MHz** (HSE 8 MHz → PLL)
2. CAN1 initialized at **500 kbps** with RX filter for ID 0x103
3. `SysTick_Handler` fires every **1 ms**:
   - Calls `HAL_SYSTICK_Callback()`
   - Packs `CAN_MotorCommand_t` into 8 bytes and transmits on CAN bus. In practice this will be sent when flight controller calculates new speed setpoint.
4. Main loop polls for incoming motor status:
   - `HAL_CAN_RxFifo0MsgPendingCallback()` unpacks received bytes into `CAN_MotorStatus_t`
   - Application reads status via `CAN_HasNewStatus()` / `CAN_GetLastStatus()`

### Blue Pill (Motor Controller)

1. System clock configured to **72 MHz** (HSE 8 MHz → PLL ×9)
2. CAN1 initialized at **500 kbps** with RX filter for ID 0x446
3. `HAL_CAN_RxFifo1MsgPendingCallback()` receives motor commands and unpacks into `CAN_MotorCommand_t`
4. Main loop processes received commands (in production: feeds into motor control algorithm)
5. `HAL_SYSTICK_Callback()` sets a flag every **10 ms** (100 Hz):
   - Main loop packs `CAN_MotorStatus_t` into 8 bytes and transmits on CAN bus

---

## Hardware Connections

### CAN Transceiver

Both nodes use an **SN65HVD230** (or equivalent 3.3V CAN transceiver) to interface with the CAN bus. 120 Ω termination resistor is required if it is not already mounted on CAN transceiver module.

**Nucleo → Transceiver → Bus:**

| Signal | STM32 NUCLEO-F446RE | SN65HVD230 | CAN Bus |
|-------:|---------------------|------------|---------|
| TX     | PA12                | TXD        | —       |
| RX     | PA11                | RXD        | —       |
| VCC    | 3.3V                | VCC        | —       |
| GND    | GND                 | GND        | —       |
| CANH   | —                   | CANH       | CANH    |
| CANL   | —                   | CANL       | CANL    |

**Blue Pill → Transceiver → Bus:**

| Signal | STM32F103C6 Blue Pill | SN65HVD230 | CAN Bus |
|-------:|-----------------------|------------|---------|
| TX     | PA12                  | TXD        | —       |
| RX     | PA11                  | RXD        | —       |
| VCC    | 3.3V                  | VCC        | —       |
| GND    | GND                   | GND        | —       |
| CANH   | —                     | CANH       | CANH    |
| CANL   | —                     | CANL       | CANL    |

**Bus termination:** 120Ω resistor between CANH and CANL at each end of the bus.

---

## File Structure

### Nucleo (CAN-LINK-NUCLEO)

#### `main.c`

- System clock configuration (180 MHz)
- CAN driver initialization and start
- `HAL_SYSTICK_Callback()` sends motor command every 1 ms
- Main loop reads motor status feedback

#### `can_driver.h / can_driver.c (Core→Inc/Src)`

CAN peripheral driver for the flight controller node.

Functions:
- `CAN_Driver_Init()`

  Initializes CAN1 peripheral (500 kbps, normal mode), configures 32-bit ID mask filter for ID 0x103 on FIFO0, and sets up TX header for motor command messages.

- `CAN_Driver_Start()`

  Starts CAN peripheral and activates RX FIFO0 message pending interrupt notification.

- `CAN_SendMotorCommand()`

  Packs `CAN_MotorCommand_t` struct into 8 bytes (big-endian) and queues for transmission.

- `CAN_HasNewStatus() / CAN_GetLastStatus()`

  Check for and retrieve the latest received motor status message.

- `HAL_CAN_RxFifo0MsgPendingCallback()`

  ISR callback that unpacks received CAN frame into `CAN_MotorStatus_t`.


---

### Blue Pill (CAN-LINK-BLUEPILL)

#### `main.c`

- System clock configuration (72 MHz)
- CAN driver initialization and start
- `HAL_SYSTICK_Callback()` sets send flag every 10 ms (100 Hz)
- Main loop processes commands and sends motor status

#### `can_driver.h / can_driver.c (Core→Inc/Src)`

CAN peripheral driver for the motor controller node.

Functions:
- `CAN_Driver_Init()`

  Initializes CAN1 peripheral (500 kbps, normal mode), configures 32-bit ID mask filter for ID 0x446 on FIFO1, and sets up TX header for motor status messages.

- `CAN_Driver_Start()`

  Starts CAN peripheral and activates RX FIFO1 message pending interrupt notification.

- `CAN_SendMotorStatus()`

  Packs `CAN_MotorStatus_t` structure into 8 bytes (big-endian) and queues for transmission.

- `CAN_HasNewCommand() / CAN_GetLastCommand()`

  Check for and retrieve the latest received motor command.

- `HAL_CAN_RxFifo1MsgPendingCallback()`

  ISR callback that unpacks received CAN frame into `CAN_MotorCommand_t`.


---

## Signal Scaling Reference

Conversion macros are provided in `can_driver.h` for both boards:

```c
// Physical to raw (for transmission)
CAN_SPEED_TO_RAW(1000.0f)   // 1000 RPM   → 10000 raw
CAN_VOLTAGE_TO_RAW(24.0f)   // 24.00 V    → 2400 raw
CAN_CURRENT_TO_RAW(1.5f)    // 1.50 A     → 150 raw

// Raw to physical (after reception)
CAN_RAW_TO_SPEED(10000)     // 10000 raw  → 1000.0 RPM
CAN_RAW_TO_VOLTAGE(2400)    // 2400 raw   → 24.00 V
CAN_RAW_TO_CURRENT(150)     // 150 raw    → 1.50 A
```

---

## Reference Materials

- **Reference Manual (Nucleo):** RM0390 Rev 7
- **Reference Manual (Blue Pill):** RM0008 Rev 21
- **Datasheet (F446RE):** DS10693 Rev 10
- **Datasheet (F103C6):** DS5791 Rev 18
- **SN65HVD230 Datasheet:** SLLS560 (Texas Instruments)
