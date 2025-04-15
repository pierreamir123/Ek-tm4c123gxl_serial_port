
#### Dependencies
- **Python Libraries**:
  - Install dependencies using:
    ```bash
    pip install pyserial matplotlib
    ```

---


###  Run Python Script
1. Ensure the Tiva board is connected to the host PC via UART.
2. Run the Python script to visualize the data:
   ```bash
   python3 plot_serial.py
   ```
3. Observe the real-time plots of distance and speed.

---

## Notes
### Firmware Configuration
- Adjust the following constants in the firmware as needed:
  - `PULSES_PER_REVOLUTION`: Number of encoder pulses per full wheel revolution.
  - `WHEEL_DIAMETER_M`: Diameter of the wheel in meters.
  - `UART_BAUDRATE`: UART communication speed (default is 115200).

### Serial Port
- Ensure the correct serial port is specified in the Python script (`/dev/ttyUSB0` by default). You can check available ports using:
  ```bash
  dmesg | grep tty
  ```

### Debugging
- If no data is received, verify the UART connections and baud rate settings.
- Use a terminal emulator (e.g., `minicom` or `screen`) to debug UART communication:
  ```bash
  screen /dev/ttyUSB0 115200
  ```

---

## Example Output
### UART Data Format
The firmware sends data in the following format:
```
Distance: X.XX m | Speed: Y.YY m/s | Dir: Z
```
Where:
- `X.XX` is the total distance traveled in meters.
- `Y.YY` is the current speed in meters per second.
- `Z` is the direction (`FWD` for forward, `REV` for reverse).



---

### 2. **Firmware/main.c**
```c
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#define PULSES_PER_REVOLUTION 360
#define WHEEL_DIAMETER_M      0.1     // meters
#define ENCODER_PIN_A         GPIO_PIN_0
#define ENCODER_PIN_B         GPIO_PIN_1
#define UART_BAUDRATE         115200

// Encoder tracking
volatile int32_t encoder_count = 0;
volatile int32_t previous_encoder_count = 0;
volatile bool direction = true;
float distance_traveled = 0.0;
float speed_mps = 0.0;

// Time tracking
uint32_t last_time_ms = 0;
uint32_t time_interval_ms = 1000; // 1 second

float wheel_circumference = 3.14159 * WHEEL_DIAMETER_M;

// ================= Timer =================
void InitTimer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000); // 1 ms period
    TimerEnable(TIMER0_BASE, TIMER_A);
}

uint32_t GetTimeMs(void) {
    return TimerValueGet(TIMER0_BASE, TIMER_A) / (SysCtlClockGet() / 1000);
}

// ================= UART =================
void InitUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), UART_BAUDRATE,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void UARTSend(const char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str++);
    }
}

// Convert float to string manually
void FloatToString(float value, char *buffer, uint8_t decimal_places) {
    int integer_part = (int)value;
    float fractional_part = value - integer_part;

    // Handle integer part
    int idx = 0;
    if (integer_part == 0) {
        buffer[idx++] = '0';
    } else {
        char temp[10];
        int i = 0;
        while (integer_part > 0) {
            temp[i++] = (integer_part % 10) + '0';
            integer_part /= 10;
        }
        while (i > 0) {
            buffer[idx++] = temp[--i];
        }
    }

    // Handle fractional part
    if (decimal_places > 0) {
        buffer[idx++] = '.';
        for (int i = 0; i < decimal_places; i++) {
            fractional_part *= 10;
            int digit = (int)fractional_part;
            buffer[idx++] = digit + '0';
            fractional_part -= digit;
        }
    }

    buffer[idx] = '\0';
}

// ================= Encoder =================
void InitEncoder(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, ENCODER_PIN_A | ENCODER_PIN_B);
    GPIOIntTypeSet(GPIO_PORTF_BASE, ENCODER_PIN_A | ENCODER_PIN_B, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE, ENCODER_PIN_A | ENCODER_PIN_B);
    IntEnable(INT_GPIOF);
    IntMasterEnable();
}

void GPIOFIntHandler(void) {
    bool A = GPIOPinRead(GPIO_PORTF_BASE, ENCODER_PIN_A);
    bool B = GPIOPinRead(GPIO_PORTF_BASE, ENCODER_PIN_B);

    if (A == B) {
        encoder_count++;
        direction = true;
    } else {
        encoder_count--;
        direction = false;
    }

    GPIOIntClear(GPIO_PORTF_BASE, ENCODER_PIN_A | ENCODER_PIN_B);
}

// ================= Calculations =================
void CalculateDistance(void) {
    float distance_per_pulse = wheel_circumference / PULSES_PER_REVOLUTION;
    distance_traveled = encoder_count * distance_per_pulse;
}

void CalculateSpeed(void) {
    uint32_t current_time_ms = GetTimeMs();
    uint32_t delta_time = current_time_ms - last_time_ms;

    if (delta_time >= time_interval_ms) {
        int32_t delta_pulses = encoder_count - previous_encoder_count;
        float distance_delta = delta_pulses * (wheel_circumference / PULSES_PER_REVOLUTION);
        speed_mps = distance_delta / (delta_time / 1000.0f);

        previous_encoder_count = encoder_count;
        last_time_ms = current_time_ms;
    }
}

// ================= Main =================
int main(void) {
    // System clock at 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitUART();
    InitEncoder();
    InitTimer();

    char msg[100];

    while (1) {
        CalculateDistance();
        CalculateSpeed();

        // Manually format the output string
        char dist_str[20], speed_str[20];
        FloatToString(distance_traveled, dist_str, 2);
        FloatToString(speed_mps, speed_str, 2);

        char *dir = direction ? "FWD" : "REV";

        // Construct the message manually
        char *msg_ptr = msg;
        const char *prefix = "Distance: ";
        const char *suffix1 = " m | Speed: ";
        const char *suffix2 = " m/s | Dir: ";
        const char *suffix3 = "\r\n";

        strcpy(msg_ptr, prefix);
        msg_ptr += strlen(prefix);

        strcpy(msg_ptr, dist_str);
        msg_ptr += strlen(dist_str);

        strcpy(msg_ptr, suffix1);
        msg_ptr += strlen(suffix1);

        strcpy(msg_ptr, speed_str);
        msg_ptr += strlen(speed_str);

        strcpy(msg_ptr, suffix2);
        msg_ptr += strlen(suffix2);

        strcpy(msg_ptr, dir);
        msg_ptr += strlen(dir);

        strcpy(msg_ptr, suffix3);

        UARTSend(msg);

        SysCtlDelay(SysCtlClockGet() / 3); // ~1 sec delay
    }
}
```

---

### 3. **Python/plot_serial.py**
```python
import serial
import matplotlib.pyplot as plt
import re

# Configure serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Data storage
distances = []
speeds = []

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        match = re.match(r"Distance: ([\d.]+) m \| Speed: ([\d.]+) m/s \| Dir: (\w+)", line)
        if match:
            distance = float(match.group(1))
            speed = float(match.group(2))
            direction = match.group(3)

            distances.append(distance)
            speeds.append(speed)

            print(f"Distance: {distance} m, Speed: {speed} m/s, Direction: {direction}")

            # Plotting
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(distances, label="Distance (m)")
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.plot(speeds, label="Speed (m/s)", color='orange')
            plt.legend()

            plt.pause(0.01)

except KeyboardInterrupt:
    ser.close()
    plt.show()
```

