

## 📌 Supported Encoder Signals

Most rotary encoders provide the following signals:

| Signal | Description |
|--------|-------------|
| A      | Channel A of quadrature signal |
| B      | Channel B of quadrature signal (90° out of phase with A) |
| IDX    | Index pulse (one pulse per revolution, optional) |

---

## 🧠 Microcontroller: Tiva TM4C123GH6PM

- Supports hardware QEI on pins:
  - **QEI0 Phase A** → **PD6**
  - **QEI0 Phase B** → **PD7**
  - **QEI0 Index** → **PD3**

---

## 🔌 Pin Mapping: Encoder to Tiva

| Encoder Signal | Tiva TM4C123GH6PM Pin | Port/Pin |
|----------------|------------------------|----------|
| VCC            | 3.3V or 5V (depends on encoder) | Power pin |
| GND            | GND                    | Ground   |
| A (Phase A)    | PD6                    | Port D, Pin 6 |
| B (Phase B)    | PD7                    | Port D, Pin 7 |
| IDX (Index)    | PD3                    | Port D, Pin 3 (optional) |

> ⚠️ If your encoder does not have an index (IDX), you can leave PD3 unconnected.

---

## 🔦 Optional: Red LED for Phase Error Detection

The code flashes the **Red LED** when a **phase error** is detected (e.g., if A and B are in phase).

| LED Color | Tiva Pin | Port/Pin |
|-----------|----------|----------|
| Red LED   | PF1      | Port F, Pin 1 |

---

## 🔧 Wiring Diagram Summary

```
Encoder         Tiva C Series (TM4C123GH6PM)
--------------------------------------------
A (Phase A) --> PD6
B (Phase B) --> PD7
IDX (Index) --> PD3 (optional)
GND         --> GND
VCC         --> 3.3V or 5V (depending on encoder voltage)
```



---

## 🛠️ Tips

- Use pull-up resistors (if needed) on encoder lines (some encoders have internal ones).
- Ensure both Tiva and encoder share a common ground.
- If the encoder direction seems reversed, swap A and B connections.

---

## 📁 File 1: `encoder_reader.c` (for Tiva TM4C123GH6PM)

```c
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "UART.h"
#include <stdio.h>

int flag = 0;

void DisableInterrupts(void);
void EnableInterrupts(void);

// Delay function
void delay(unsigned long millisecs) {
    unsigned long count;
    while(millisecs > 0 ) { 
        count = 333333; 
        while (count > 0) { 
            count--;
        } 
        millisecs--;
    }
}

// Port F Initialization (Red LED on PF1)
void PortF_Init(void){ 
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000020;     // Port F Clock Turned ON
    delay = SYSCTL_RCGC2_R;           // Delay for clock turn ON  
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // Unlock PortF   
    GPIO_PORTF_CR_R |= 0x02;          // Allow changes to PF1       
    GPIO_PORTF_AMSEL_R &= ~0x02;      // Disable analog function on PF1
    GPIO_PORTF_PCTL_R &= 0x00000000;  // GPIO clear bit PCTL  
    GPIO_PORTF_DIR_R |= 0x02;         // PF1 set as output  
    GPIO_PORTF_AFSEL_R &= ~0x02;      // No alternate function
    GPIO_PORTF_PUR_R &= ~0x02;        // Disable pullup resistors on PF1       
    GPIO_PORTF_DEN_R |= 0x02;         // Enable digital function on PF1        
}

// UART Initialization
void UART_Init(void){
    SYSCTL_RCGCUART_R |= 0x01;     // Enable UART0
    SYSCTL_RCGCGPIO_R |= 0x01;     // Enable GPIOA

    while((SYSCTL_PRGPIO_R & 0x01) == 0); // Wait for peripheral to be ready

    // Configure PA0 and PA1 as UART
    GPIO_PORTA_AMSEL_R &= ~0x03;
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) | 0x00000011;
    GPIO_PORTA_DIR_R |= 0x02;
    GPIO_PORTA_DIR_R &= ~0x01;
    GPIO_PORTA_AFSEL_R |= 0x03;
    GPIO_PORTA_DEN_R |= 0x03;

    UART0_CTL_R &= ~0x01;          // Disable UART

    // Baud rate: 115200 with 80 MHz clock
    UART0_IBRD_R = 43;             // Integer part
    UART0_FBRD_R = 25;             // Fractional part

    UART0_LCRH_R = (0x3 << 5);     // 8-bit word length
    UART0_CTL_R = 0x301;           // Enable TX, RX, and UART
}

// Send a string via UART
void UART_OutString(char *str) {
    while (*str) {
        while ((UART0_FR_R & 0x20) != 0); // Wait until TX FIFO not full
        UART0_DR_R = *str;                // Send character
        str++;
    }
}

// QEI Initialization
void QEI_Init() {
    SYSCTL_RCGCQEI_R |= 0x01;       // QEI0 run mode clock gating control on
    SYSCTL_RCGCGPIO_R |= 0x08;      // Enable clock for PORTD

    GPIO_PORTD_LOCK_R = 0x4C4F434B; // Unlock port D for changes
    GPIO_PORTD_CR_R |= 0xC8;        // Allow changes to PD3,6,7      
    GPIO_PORTD_AMSEL_R &= 0x00;     // Disable analog function
    GPIO_PORTD_DEN_R |= 0xC8;       // Enable Digital Pins
    GPIO_PORTD_DIR_R &= ~0xC8;      // Configure direction of pins as Input
    GPIO_PORTD_AFSEL_R |= 0xC8;     // QEI alternate function on PD3,6,7
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x00FF0FFF)+0x66006000; 

    NVIC_PRI3_R = (NVIC_PRI3_R&0xFFFF1FFF)|0xA000; // Priority 5
    NVIC_EN0_R |= 0x2000;                          // Enable QEI0 interrupt

    QEI0_CTL_R |= 0x38; 
    QEI0_INTEN_R |= 0x08;              // Phase error interrupt enable
    QEI0_LOAD_R = 0x00001387;          // Velocity timer value
    QEI0_MAXPOS_R = ((1024 * 4) - 1);  // Max position value
    QEI0_CTL_R |= 0x01;                // Enable QEI
}

// Quadrature Encoder Interrupt Handler
void Quadrature0_Handler(void) {
    if((QEI0_RIS_R&0x08)) {
        flag = 1;
        QEI0_ISC_R |= 0x08; // Clear interrupt
    }
}

int main(void) {
    unsigned long position;
    char buffer[32];

    PLL_Init(); // Set system clock to 80 MHz
    UART_Init();
    QEI_Init();
    PortF_Init();

    while(1) {
        if(flag) {
            GPIO_PORTF_DATA_R |= 0x02;
            delay(1);
            flag = 0;
        } else {
            position = QEI0_POS_R & 0x00000FFF;
            sprintf(buffer, "POS:%lu\n", position);
            UART_OutString(buffer);
            delay(100); // ~10 Hz update rate
        }

        GPIO_PORTF_DATA_R &= ~0x02;
        EnableInterrupts();
    }
}
```

---

## 📁 File 2: `read_encoder.py` (for Raspberry Pi)

```python
import serial
import time

# Adjust the port accordingly
SERIAL_PORT = '/dev/ttyUSB0'  # Or '/dev/ttyAMA0' or '/dev/ttyS0'
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("POS:"):
                    pos = line[4:]  # Extract number after "POS:"
                    print(f"Encoder Position: {pos}")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
```

---

## ✅ Usage Instructions

### For Tiva Side:

1. Compile and flash `encoder_reader.c` using:
   - Code Composer Studio (CCS)
   - ARM GCC Toolchain
   - Or any IDE supporting TM4C123GH6PM

3. Ensure correct baud rate is set in both ends (115200).

---

### For Raspberry Pi Side:

1. Install dependencies:
```bash
pip install pyserial
```

2. Run the Python script:
```bash
python read_encoder.py
```

---

## 🎯 Output Example

```
Listening on /dev/ttyUSB0 at 115200 baud...
Encoder Position: 1023
Encoder Position: 1024
Encoder Position: 1027
Encoder Position: 1031
...
```

