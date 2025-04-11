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