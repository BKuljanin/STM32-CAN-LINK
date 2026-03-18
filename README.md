# STM32 IMU Sensor Fusion + SD Card Logging
### MPU6500 SPI + DMA + Complementary Filter + FATFS (NUCLEO-F446RE)

**Note:**  
This project combines a bare-metal SPI + DMA IMU driver with real-time sensor fusion and SD card data logging.  
The MPU6500 driver is fully register-level (**SPI1 + DMA2**), while SD logging uses **FATFS over SPI2**.

---

## This project demonstrates

- Bare-metal **SPI1** configuration (register-level)
- Full-duplex SPI burst reads using **DMA** (TX+RX)
- Data Ready interrupt → start DMA transfer (**event-driven sampling**)
- DMA Transfer Complete ISR → stop DMA, release CS, parse data
- IMU initialization + IMU calibration (bias estimation)
- **Complementary filter** sensor fusion (roll + pitch)
- **SD card logging** via FATFS over SPI2 (CSV output)
- Task based scheduling (**no RTOS**)

---

## Output Signals

This project produces:

- Accelerometer data (X/Y/Z) in **g**
- Gyroscope data (X/Y/Z) in **deg/s**
- Temperature in **°C**
- Estimated attitude:
  - Roll in **rad** (logged in deg)
  - Pitch in **rad** (logged in deg)

> Yaw is not estimated (no magnetometer).

---

## Overview

The MPU6500 provides a continuous block of sensor output registers starting at **0x3B**:

- ACCEL_XOUT_H .. ACCEL_ZOUT_L (6 bytes)
- TEMP_OUT_H .. TEMP_OUT_L (2 bytes)
- GYRO_XOUT_H .. GYRO_ZOUT_L (6 bytes)

In this project **14 bytes** are read in a single burst read starting at **0x3B**.

---

## Sampling Rate and Timing

- IMU sampling is configured to **~1 kHz** (Data Ready interrupt).
- Complementary filter update runs whenever a new sample is available (**~1 kHz**).
- SD logging runs slower (e.g. **50 ms** → 20 Hz) to keep file size reasonable.

## High-Level Flow

1) IMU acquisition (SPI1 + DMA2, event-driven)

- MPU6500 INT pin rising edge occurs when new sensor data is ready (Data Ready interrupt).
- STM32 EXTI interrupt fires (PC13 in this project).
- In EXTI15_10_IRQHandler():
    - configure DMA NDTR + buffer addresses
    - start DMA streams (TX and RX)
- SPI clocks out:
    - the read register address (to configure slave internal pointer)
    - dummy bytes to clock in the remaining sensor bytes (TX)
- DMA RX Stream Transfer Complete interrupt fires.
- In DMA2_Stream2_IRQHandler():
    - clear DMA flags
    - disable DMA streams
    - bring CS high to disable slave
    - parse raw data → apply scaling and bias → update MPU6500_Data_t

2) Sensor fusion task

- imu_process_task() runs in the main loop (no RTOS).
- If imu_seq changed → new sample exists:
    - copy IMU data + attitude state locally
    - run complementary filter
    - publish updated attitude

3) SD logging task (main loop)

- Every logging_task_timing_ms (e.g. 50 ms):
    - log one CSV line:
    - time
    - accel/gyro/temp
    - roll/pitch
 
---

## Hardware Connections
IMU: MPU6500

| Signal | MPU6500 | STM32 NUCLEO-F446RE |
|-------:|---------|---------------------|
| VCC    | VCC     | 3.3V                |
| GND    | GND     | GND                 |
| SCLK   | SCL/SCLK| PA5 (SPI1_SCK)      |
| MOSI   | SDA/SDI | PA7 (SPI1_MOSI)     |
| MISO   | AD0/SDO | PA6 (SPI1_MISO)     |
| SS     | NCS     | PA9 (GPIO Output)   |
| INT    | INT     | PC13 (GPIO Input)   |

---

MicroSD Card SPI Module (generic 6-pin SPI breakout with onboard 3.3V regulator and level shifting)
| Signal | SD card module | STM32 NUCLEO-F446RE |
|-------:|---------|---------------------|
| VCC    | VCC     | 5V                |
| GND    | GND     | GND                 |
| SCLK   | SCL/SCLK| PB10 (SPI2_SCK)      |
| MOSI   | MOSI | PC1 (SPI2_MOSI)     |
| MISO   | MISO | PC2 (SPI2_MISO)     |
| SS     | CS     | PC3 (GPIO Output)   |

---

## Complementary Filter (Roll + Pitch)

This project uses a **complementary filter** to estimate roll and pitch by combining:

- **Gyroscope integration** (good short-term, but drifts over time due to bias)
- **Accelerometer tilt estimate** (stable long-term, but noisy under vibration/linear acceleration)

The filter runs whenever a new IMU sample is available.

Note: In this project we use the roll–pitch (X–Y) rotation order, meaning roll is computed first about the body X-axis and pitch second about the body Y-axis, because this ordering matches the standard aircraft convention.

### Accelerometer Tilt Estimation

Roll angle from accelerometer:

$$ ϕ_{acc} = atan2(\frac{a_{y}}{a_{z}}) $$

Pitch angle from accelerometer:

$$\theta_{acc} = atan2\left(\frac{-a_x}{\sqrt{a_y^2 + a_z^2}}\right)$$

---

### Gyroscope Integration

The gyroscope measures angular velocity in degrees per second (deg/s).  
To obtain angle, we integrate over time:

$$ ϕ_{gyro} = ϕ_{prev} +  ω_{x} * dt $$

$$ θ_{gyro} = θ_{prev} +  ω_{y} * dt $$

Where:

- dt is the time step in seconds  
- angular velocities are in deg/s but they are converted to rad/s before the above calculation

---

### Complementary Filter Fusion

The complementary filter combines gyro and accelerometer estimates:

$$ ϕ = α * ϕ_{gyro}+ (1-α) ϕ_{acc} $$

$$ θ = α * θ_{gyro}+ (1-α) θ_{acc} $$

Where α is in range 0.95~0.98. This project uses α=0.98.

- High α → trust gyroscope more  
  - Better response to fast motion  
  - More long-term drift  

- Low α → trust accelerometer more  
  - Better long-term stability  
  - More noise during motion  

---

### Data Units and Conventions

- Accelerometer `a_x, a_y, a_z` are in **g**
- Gyroscope `omega_x, omega_y, omega_z` are in **deg/s**
- Internal attitude state (`roll`, `pitch`) is stored in **radians**
- Logged attitude is converted to **degrees**

---

## File Structure

### `main.c`

- GPIO initialization
- SPI1 in DMA mode initialization for IMU reading
- SPI2 in DMA mode for SD card writing
- Initialization of log file on SD card
- USART2 initialization for debugging
- MPU6500 initialization
- IMU calibration
- EXTI configuration for Data Ready
- Interrupts used:
  
    -EXTI15_10_IRQHandler() → Data Ready triggers a DMA read
  
    -DMA2_Stream2_IRQHandler() → RX DMA transfer complete
  
- Calls sensor fusion task
- Calls SD card logging task

---

### `mpu6500.h / mpu6500.c (Core->Inc/Src)`
Initializes MPU6500 IMU sensor, reads it, and writes to it using **SPI** in **DMA** mode.

Functions:
- `mpu6500_init();`
  
  Initializes the sensor by waking it up and configuring the measurement range for gyroscope and accelerometer. Configures LP filter and 1kHz frequency and enables Data Ready Interrupt pin.
  
- `mpu6500_read(...);`

  Starts a full-duplex SPI DMA burst read (TX dummy bytes + RX buffer fill).

- `mpu6500_write(...);`

  Starts a full-duplex SPI DMA burst write (RX dummy bytes + TX buffer fill).

- `mpu6500_read_blocking() / mpu6500_write_blocking();`

   SPI blocking read and write, used only for IMU initialization and calibration. In real-time only DMA SPI is used for data collection.

- `mpu6500_calibrate_imu(...);`

  Reading gyroscope and acclerometer info for given number of samples and finds average. The sensor is still while this is in progress. This value is used in processing in mpu6500_process, bias is subtracted.

- `dma_callback(...);`

  Handles disabling SPI slave and DMA transfer on DMA completed interrupt. Calls processing function.

---

### `spi.h / spi.c (Core->Inc/Src)`
Configures **SPI** peripheral in **DMA** mode.

- `dma2_stream_2_3_init();`

  Initializing DMA2 Stream 2 Channel 3 for SPI_RX. Initializing DMA2 Stream 3 Channel 3 for SPI_TX.

- `dma2_enable()/dma2_disable;`

  Enable and disable DMA2 helper functions.

- `set_dma_transfer_length()/set_dma_source();`
 
  Sets DMA transfer length and DMA memory sources for transmit and receive.

- `spi_gpio_init();`
 
  Initializes GPIO pins for SPI protocol.

- `spi1_config();`
 
  Initializes SPI1 peripheral.

- `cs_enable()/cs_disable();`
 
  Pulling CS line low to select the slave, pulls high to disable the slave.

- `dma2_clear_spi1_flags;`
 
  Clear all pending interrupt flags (TC, HT, TE, DME, FE) for DMA2 Stream2 and Stream3.

---

### `exti.h / exti.c (Core->Inc/Src)`

 - `pc13_exti_init();`
 
  Initializes PC13 as input. Enabling rising edge interrupt. Using it for Data Ready Interrupt.
 
---

### `complementary_filter.h / complementary_filter.c (Core->Inc/Src)`

 - `imu_process_task();`
 
  Creates local copy of last sample, checks if there is new data and calls complementary filter (see above for details).
 
---

## SD Card Logging Module (`logger.c / logger.h (Core->Inc/Src)`)

This module implements a logging interface built on top of **FATFS**.  
It provides:

- File creation
- Buffered writes
- Safe closing with synchronization
- Structured CSV logging of IMU + attitude data

---

## Reference Materials
All register settings, timer configurations, and GPIO modes in this code are implemented based on:

- **Reference Manual:** RM0390 Rev 7  
- **Datasheet:** DS10693 Rev 10  
- **User Manual:** UM1724 Rev 17
- **Cortex M4 Generic User Guide:** DUI 0553A

  
* **Sensor Documentation:** MPU-6500 Register Map and Descriptions Revision 2.1
