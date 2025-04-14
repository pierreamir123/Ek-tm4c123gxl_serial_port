#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
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
uint8_t IntToString(int16_t value, char *str);
void FloatToString(float value, char *str, uint8_t decimal_places);

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    I2C_Init();
    UART_Init();

    // Wake up MPU6050
    I2C_WriteByte(0x6B, 0x00);

    while (1) {
        // Read accelerometer data (X, Y, Z)
        int16_t ax = (I2C_ReadByte(0x3B) << 8) | I2C_ReadByte(0x3C);
        int16_t ay = (I2C_ReadByte(0x3D) << 8) | I2C_ReadByte(0x3E);
        int16_t az = (I2C_ReadByte(0x3F) << 8) | I2C_ReadByte(0x40);

        // Read gyroscope data (X, Y, Z)
        int16_t gx = (I2C_ReadByte(0x43) << 8) | I2C_ReadByte(0x44);
        int16_t gy = (I2C_ReadByte(0x45) << 8) | I2C_ReadByte(0x46);
        int16_t gz = (I2C_ReadByte(0x47) << 8) | I2C_ReadByte(0x48);

        // Read temperature data
        int16_t temp_raw = (I2C_ReadByte(0x41) << 8) | I2C_ReadByte(0x42);
        float temp = (temp_raw / 340.0f) + 36.53f;

        // Calculate pitch and roll
        float pitch, roll;
        float ax_f = (float)ax, ay_f = (float)ay, az_f = (float)az;

        // Convert raw accelerometer values to g-force (optional scaling)
        const float sensitivity = 16384.0f; // For ±2g range
        ax_f /= sensitivity;
        ay_f /= sensitivity;
        az_f /= sensitivity;

        // Calculate pitch and roll in degrees
        pitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * (180.0f / M_PI);
        roll = atan2(ay_f, az_f) * (180.0f / M_PI);

        // Manually construct the string
        char buffer[128];
        uint8_t index = 0;

        const char accel_prefix[] = "Accel: ";
        for (uint8_t i = 0; i < sizeof(accel_prefix) - 1; i++) {
            buffer[index++] = accel_prefix[i];
        }

        index += IntToString(ax, &buffer[index]);
        buffer[index++] = ',';
        index += IntToString(ay, &buffer[index]);
        buffer[index++] = ',';
        index += IntToString(az, &buffer[index]);

        const char gyro_prefix[] = " Gyro: ";
        for (uint8_t i = 0; i < sizeof(gyro_prefix) - 1; i++) {
            buffer[index++] = gyro_prefix[i];
        }

        index += IntToString(gx, &buffer[index]);
        buffer[index++] = ',';
        index += IntToString(gy, &buffer[index]);
        buffer[index++] = ',';
        index += IntToString(gz, &buffer[index]);

        const char temp_prefix[] = " Temp: ";
        for (uint8_t i = 0; i < sizeof(temp_prefix) - 1; i++) {
            buffer[index++] = temp_prefix[i];
        }

        FloatToString(temp, &buffer[index], 2);
        index += strlen(&buffer[index]);

        const char pitch_prefix[] = " Pitch: ";
        for (uint8_t i = 0; i < sizeof(pitch_prefix) - 1; i++) {
            buffer[index++] = pitch_prefix[i];
        }

        FloatToString(pitch, &buffer[index], 2);
        index += strlen(&buffer[index]);

        const char roll_prefix[] = " Roll: ";
        for (uint8_t i = 0; i < sizeof(roll_prefix) - 1; i++) {
            buffer[index++] = roll_prefix[i];
        }

        FloatToString(roll, &buffer[index], 2);
        index += strlen(&buffer[index]);

        buffer[index++] = '\n';
        buffer[index] = '\0';

        UART_SendString(buffer);
        SysCtlDelay(SysCtlClockGet() / 10); // Delay for 100ms
    }
}

// Helper functions remain unchanged