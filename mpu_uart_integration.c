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

        // Send data without using snprintf
        UART_SendString("Accel: ");
        
        char numBuffer[8];
        // Convert ax to string
        itoa((int)ax, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(",");
        
        // Convert ay to string
        itoa((int)ay, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(",");
        
        // Convert az to string
        itoa((int)az, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(" Gyro: ");
        
        // Convert gx to string
        itoa((int)gx, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(",");
        
        // Convert gy to string
        itoa((int)gy, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(",");
        
        // Convert gz to string
        itoa((int)gz, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(" Temp: ");
        
        // Convert temp to string (simple conversion for 2 decimal places)
        int temp_int = (int)temp;
        int temp_frac = (int)((temp - temp_int) * 100);
        itoa(temp_int, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString(".");
        if (temp_frac < 10) {
            UART_SendString("0");
        }
        itoa(temp_frac, numBuffer, 10);
        UART_SendString(numBuffer);
        UART_SendString("\n");
        
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