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

// ================= Custom String Functions =================
// Custom strlen implementation
uint8_t MyStrlen(const char *str) {
    uint8_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

// Custom strcpy implementation
void MyStrcpy(char *dest, const char *src) {
    uint8_t i = 0;
    while (src[i] != '\0') {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0'; // Null-terminate the destination string
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

        // Prefix: "Distance: "
        const char prefix[] = "Distance: ";
        MyStrcpy(msg_ptr, prefix);
        msg_ptr += MyStrlen(prefix);

        // Distance value
        MyStrcpy(msg_ptr, dist_str);
        msg_ptr += MyStrlen(dist_str);

        // Suffix 1: " m | Speed: "
        const char suffix1[] = " m | Speed: ";
        MyStrcpy(msg_ptr, suffix1);
        msg_ptr += MyStrlen(suffix1);

        // Speed value
        MyStrcpy(msg_ptr, speed_str);
        msg_ptr += MyStrlen(speed_str);

        // Suffix 2: " m/s | Dir: "
        const char suffix2[] = " m/s | Dir: ";
        MyStrcpy(msg_ptr, suffix2);
        msg_ptr += MyStrlen(suffix2);

        // Direction
        MyStrcpy(msg_ptr, dir);
        msg_ptr += MyStrlen(dir);

        // Suffix 3: "\r\n"
        const char suffix3[] = "\r\n";
        MyStrcpy(msg_ptr, suffix3);

        // Send the message over UART
        UARTSend(msg);

        SysCtlDelay(SysCtlClockGet() / 3); // ~1 sec delay
    }
}