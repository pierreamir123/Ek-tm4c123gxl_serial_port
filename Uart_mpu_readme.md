# Serial Communication with MPU6050 and EK-TM4C123GXL

This project demonstrates how to read data from an **MPU6050 IMU** (Inertial Measurement Unit) using the **EK-TM4C123GXL (Tiva C Series LaunchPad)** and send it over a serial connection to a **Raspberry Pi**. The Raspberry Pi reads and processes the data, which includes accelerometer, gyroscope, and temperature readings.

---

## Table of Contents
1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Hardware Setup](#hardware-setup)
4. [Programming the EK-TM4C123GXL](#programming-the-ek-tm4c123gxl)
5. [Setting Up the Raspberry Pi](#setting-up-the-raspberry-pi)
6. [Running the System](#running-the-system)
7. [Troubleshooting](#troubleshooting)

---

## Overview
The **MPU6050** is a popular IMU that provides:
- Accelerometer data (X, Y, Z axes)
- Gyroscope data (X, Y, Z axes)
- Temperature data

The **EK-TM4C123GXL** communicates with the MPU6050 via I2C, formats the data into a string, and sends it over UART to the Raspberry Pi. A Python script on the Raspberry Pi reads and parses the serial data for further processing or visualization.

---

## Hardware Setup
Connect the MPU6050 to the EK-TM4C123GXL as follows:

| MPU6050 Pin | EK-TM4C123GXL Pin |
|-------------|-------------------|
| VCC         | 3.3V              |
| GND         | GND               |
| SDA         | PB3 (I2C Data)    |
| SCL         | PB2 (I2C Clock)   |

Ensure the EK-TM4C123GXL is powered on and connected to the Raspberry Pi via USB for serial communication.

---

## Programming the EK-TM4C123GXL
The EK-TM4C123GXL reads data from the MPU6050 via I2C and sends it over UART. Below is the firmware code for the EK-TM4C123GXL.

### Firmware Code
```c
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"

#define MPU6050_ADDR 0x68  // Default I2C address of MPU6050

// Function prototypes
void I2C_Init(void);
void UART_Init(void);
uint8_t I2C_ReadByte(uint8_t reg);
void I2C_WriteByte(uint8_t reg, uint8_t value);
void UART_SendString(const char *str);

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    I2C_Init();
    UART_Init();
    
    // Send initialization message to confirm UART connection is active
    UART_SendString("MPU6050 UART Connection Initialized\r\n");
    UART_SendString("System ready and waiting for data...\r\n");
    
    // Wake up MPU6050
    I2C_WriteByte(0x6B, 0x00);
    UART_SendString("MPU6050 sensor activated\r\n");

    while (1) {
        int16_t ax = (I2C_ReadByte(0x3B) << 8) | I2C_ReadByte(0x3C);
        int16_t ay = (I2C_ReadByte(0x3D) << 8) | I2C_ReadByte(0x3E);
        int16_t az = (I2C_ReadByte(0x3F) << 8) | I2C_ReadByte(0x40);

        int16_t gx = (I2C_ReadByte(0x43) << 8) | I2C_ReadByte(0x44);
        int16_t gy = (I2C_ReadByte(0x45) << 8) | I2C_ReadByte(0x46);
        int16_t gz = (I2C_ReadByte(0x47) << 8) | I2C_ReadByte(0x48);

        int16_t temp_raw = (I2C_ReadByte(0x41) << 8) | I2C_ReadByte(0x42);
        float temp = (temp_raw / 340.0f) + 36.53f;

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Accel: %d,%d,%d Gyro: %d,%d,%d Temp: %.2f\n",(int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz, temp);

        UART_SendString(buffer);
        SysCtlDelay(SysCtlClockGet() / 10);
    }
}

void I2C_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

void UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

uint8_t I2C_ReadByte(uint8_t reg) {
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE));

    return I2CMasterDataGet(I2C0_BASE);
}

void I2C_WriteByte(uint8_t reg, uint8_t value) {
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, value);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE));
}

void UART_SendString(const char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str++);
    }
}
```

Flash this code onto the EK-TM4C123GXL using your preferred development environment (e.g., Code Composer Studio).

---

## Setting Up the Raspberry Pi
1. Install the `pyserial` library:
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip3 install pyserial
   ```

2. Create a Python script (`read_serial.py`) to read and parse the serial data:

```python
import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your actual port (e.g., /dev/ttyACM0)
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print(f"Received: {data}")

            if data.startswith("Accel:") and "Gyro:" in data and "Temp:" in data:
                parts = data.split()
                accel = parts[1].split(",")
                gyro = parts[3].split(",")
                temp = float(parts[5])

                print(f"Accelerometer: X={accel[0]}, Y={accel[1]}, Z={accel[2]}")
                print(f"Gyroscope: X={gyro[0]}, Y={gyro[1]}, Z={gyro[2]}")
                print(f"Temperature: {temp}°C")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
```

---

## Running the System
1. Flash the firmware onto the EK-TM4C123GXL.
2. Run the Python script on the Raspberry Pi:
   ```bash
   python3 read_serial.py
   ```

Expected output:
```
Connected to /dev/ttyUSB0 at 115200 baud.
Received: Accel: 123,-456,789 Gyro: -101,202,-303 Temp: 25.67
Accelerometer: X=123, Y=-456, Z=789
Gyroscope: X=-101, Y=202, Z=-303
Temperature: 25.67°C
```

---

## Troubleshooting
1. **I2C Issues**: Use an I2C scanner to verify the MPU6050 address (`0x68`).
2. **Serial Port Issues**: Double-check the serial port name (`/dev/ttyUSB0` or `/dev/ttyACM0`) and permissions.
3. **Data Parsing Errors**: Ensure the transmitted string format matches the parsing logic in the Python script.
