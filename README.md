Here’s a professional and clear **README** file for your project that explains the setup, steps, and usage of the serial communication between the Raspberry Pi and the EK-TM4C123GXL.

---

# Serial Communication Between Raspberry Pi and EK-TM4C123GXL

This project demonstrates how to establish a serial communication link between a **Raspberry Pi** and an **EK-TM4C123GXL (Tiva C Series LaunchPad)**. The EK-TM4C123GXL sends the string `"welcome home"` over UART via a USB connection, and the Raspberry Pi reads and displays the message using a Python script.

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Hardware Setup](#hardware-setup)
3. [Programming the EK-TM4C123GXL](#programming-the-ek-tm4c123gxl)
4. [Setting Up the Raspberry Pi](#setting-up-the-raspberry-pi)
5. [Running the Python Script](#running-the-python-script)
6. [Troubleshooting](#troubleshooting)

---

## Prerequisites
Before starting, ensure you have the following:
- **EK-TM4C123GXL (Tiva C Series LaunchPad)**
- **Raspberry Pi** (with Raspbian OS or any Linux-based OS installed)
- **USB Cable** to connect the EK-TM4C123GXL to the Raspberry Pi
- **Development Environment** for programming the EK-TM4C123GXL (e.g., Code Composer Studio or Keil)
- Basic knowledge of Python and embedded systems

---

## Hardware Setup
1. Connect the **EK-TM4C123GXL** to the **Raspberry Pi** using a USB cable.
   - The USB connection will create a virtual COM port on the Raspberry Pi.
2. Ensure both devices are powered on.

---

## Programming the EK-TM4C123GXL
The EK-TM4C123GXL needs to be programmed to send the string `"welcome home"` over its UART interface.

### Steps:
1. Install **TivaWare** and set up your development environment (e.g., Code Composer Studio).
2. Write the following code to configure UART0 and send the string:

```c
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

int main(void) {
    // Set system clock to 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable UART0 and GPIOA peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure PA0 (RX) and PA1 (TX) for UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UART0 for 115200 baud rate, 8-N-1
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Send the string "welcome home" repeatedly
    while (1) {
        UARTSend((const uint8_t *)"welcome home\n", 13); // 13 characters including newline
        SysCtlDelay(SysCtlClockGet() / 3);               // Delay ~1 second
    }
}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count) {
    while (ui32Count--) {
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}
```

3. Compile and flash the code onto the EK-TM4C123GXL.

---

## Setting Up the Raspberry Pi
1. Identify the virtual COM port created by the USB connection:
   ```bash
   ls /dev/tty*
   ```
   Look for `/dev/ttyUSB0` or `/dev/ttyACM0`.

2. Install the required Python library (`pyserial`):
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip3 install pyserial
   ```

3. Create a Python script (`read_serial.py`) to read data from the serial port:

```python
import serial
import time

# Define the serial port and baud rate
SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your actual port (e.g., /dev/ttyACM0)
BAUD_RATE = 115200

# Open the serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# Read and print data from the serial port
try:
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            data = ser.readline().decode('utf-8').strip()  # Read and decode the data
            print(f"Received: {data}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
```

---

## Running the Python Script
1. Run the Python script on the Raspberry Pi:
   ```bash
   python3 read_serial.py
   ```

2. If everything is set up correctly, you should see output like this:
   ```
   Connected to /dev/ttyUSB0 at 115200 baud.
   Received: welcome home
   Received: welcome home
   ...
   ```

---

## Troubleshooting
1. **Incorrect Port**: If the script cannot find the serial port, double-check the output of `ls /dev/tty*` and update the `SERIAL_PORT` variable in the script.
2. **Baud Rate Mismatch**: Ensure the baud rate in the Python script matches the baud rate configured in the EK-TM4C123GXL firmware (115200 in this case).
3. **Permissions Issue**: If you encounter a permission error, add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Then log out and log back in.

---

## Conclusion
This project successfully demonstrates serial communication between the Raspberry Pi and the EK-TM4C123GXL. You can expand this setup for more complex applications, such as sending sensor data or controlling actuators.

For questions or issues, feel free to open an issue in this repository or contact the author.

---

**Happy Coding!** 🚀