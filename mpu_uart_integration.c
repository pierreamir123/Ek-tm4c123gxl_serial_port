#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#define MPU6050_ADDR 0x68  // Default I2C address of MPU6050

// Global variables
uint32_t g_ui32SysClock;
char g_receiveBuffer[128];
volatile uint32_t g_receiveIndex = 0;
volatile bool g_receiveComplete = false;

// Function prototypes
void I2C_Init(void);
void UART_Init(void);
uint8_t I2C_ReadByte(uint8_t reg);
void I2C_WriteByte(uint8_t reg, uint8_t value);
void IntToStr(int value, char *buffer);
void UARTIntHandler(void);

int main(void) {
    // Set the clocking to run directly from the crystal
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    g_ui32SysClock = SysCtlClockGet();
    
    I2C_Init();
    UART_Init();
    
    // Enable processor interrupts
    ROM_IntMasterEnable();
    
    // Wake up MPU6050
    I2C_WriteByte(0x6B, 0x00);

    while (1) {
        int16_t ax = (I2C_ReadByte(0x3B) << 8) | I2C_ReadByte(0x3C);
        int16_t ay = (I2C_ReadByte(0x3D) << 8) | I2C_ReadByte(0x3E);
        int16_t az = (I2C_ReadByte(0x3F) << 8) | I2C_ReadByte(0x40);

        int16_t gx = (I2C_ReadByte(0x43) << 8) | I2C_ReadByte(0x44);
        int16_t gy = (I2C_ReadByte(0x45) << 8) | I2C_ReadByte(0x46);
        int16_t gz = (I2C_ReadByte(0x47) << 8) | I2C_ReadByte(0x48);

        int16_t temp_raw = (I2C_ReadByte(0x41) << 8) | I2C_ReadByte(0x42);
        float temp = (temp_raw / 340.0f) + 36.53f;

        // Check if we received any commands
        if (g_receiveComplete) {
            g_receiveComplete = false;
            g_receiveIndex = 0;
        }
        
        SysCtlDelay(g_ui32SysClock / 10);
    }
}

// Custom integer to string conversion function to replace itoa
void IntToStr(int value, char *buffer) {
    // Handle negative numbers
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }
    
    // Find the number of digits
    int temp = value;
    int numDigits = 0;
    
    if (value == 0) {
        numDigits = 1;
    } else {
        while (temp > 0) {
            temp /= 10;
            numDigits++;
        }
    }
    
    // Place the null terminator
    buffer[numDigits] = '\0';
    
    // Fill in the digits from right to left
    int i = numDigits - 1;
    if (value == 0) {
        buffer[0] = '0';
    } else {
        while (value > 0) {
            buffer[i--] = '0' + (value % 10);
            value /= 10;
        }
    }
}

void I2C_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    I2CMasterInitExpClk(I极I2C0_BASE, SysCtlClockGet(), false);
}

void UART_Init(void) {
    // Enable the peripherals used by this example
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    // Set GPIO A0 and A1 as UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Configure the UART for 115,200, 8-N-1 operation
    ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    
    // Enable the UART interrupt
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
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

// The UART interrupt handler
void UARTIntHandler(void) {
    uint32_t ui32Status;

    // Get the interrupt status
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO
    while (ROM_UARTCharsAvail(UART0_BASE)) {
        // Read the next character from the UART
        char receivedChar = ROM_UARTCharGetNonBlocking(UART0_BASE);
        
        // Store the character in our buffer
        if (g_receiveIndex < sizeof(g_receiveBuffer) - 1) {
            g_receiveBuffer[g_receiveIndex++] = receivedChar;
            
            // Check if this is the end of a command (newline)
            if (receivedChar == '\n' || receivedChar == '\r') {
                g_receiveBuffer[g_receiveIndex] = '\0';  // Null terminate
                g_receiveComplete = true;
            }
        }
    }
}