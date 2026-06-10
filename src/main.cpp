// Standard libraries
#include <stdlib.h>
#include <stdio.h>

// ESP32 libraries
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Arduino libraries
#include <Arduino.h>
#include <SimpleFOC.h>

// Custom system libraries
#include "Gimbal.h"
#include "Motor.h"
#include "AS5048AEncoder.h"
#include "LSM6DSV16X_IMU.h"
#include "SPIBus.h"

// Pin definitions
#define PIN_SPI_MOSI                35
#define PIN_SPI_MISO                37
#define PIN_SPI_SCLK                36
#define PIN_SPI_IMU_CS              34
#define PIN_SPI_ENC_YAW_CS          9
#define PIN_SPI_ENC_PITCH_CS        8
#define PIN_SPI_ENC_ROLL_CS         7
#define PIN_MOTOR_YAW_PWM_1         10
#define PIN_MOTOR_YAW_PWM_2         11
#define PIN_MOTOR_YAW_PWM_3         12
#define PIN_MOTOR_PITCH_PWM_1       13
#define PIN_MOTOR_PITCH_PWM_2       14
#define PIN_MOTOR_PITCH_PWM_3       15
#define PIN_MOTOR_ROLL_PWM_1        16
#define PIN_MOTOR_ROLL_PWM_2        17
#define PIN_MOTOR_ROLL_PWM_3        18
// Add BMS and power management stuff later
// #define 

// Initialisation of classes
SPIBus  spi_bus_2(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK);
LSM6DSV16X_IMU imu(PIN_SPI_IMU_CS);
AS5048AEncoder yaw_enc(PIN_SPI_ENC_YAW_CS);
AS5048AEncoder pitch_enc(PIN_SPI_ENC_PITCH_CS);
AS5048AEncoder roll_enc(PIN_SPI_ENC_ROLL_CS);
Motor yaw_motor(PIN_MOTOR_YAW_PWM_1, PIN_MOTOR_YAW_PWM_2, PIN_MOTOR_YAW_PWM_3);
Motor pitch_motor(PIN_MOTOR_PITCH_PWM_1, PIN_MOTOR_PITCH_PWM_2, PIN_MOTOR_PITCH_PWM_3);
Motor roll_motor(PIN_MOTOR_ROLL_PWM_1, PIN_MOTOR_ROLL_PWM_2, PIN_MOTOR_ROLL_PWM_3);
Gimbal gimbal(&spi_bus_2, &imu, &yaw_enc, &pitch_enc, &roll_enc);

// -------- CAN OPTIMISE INTEGER SIZES LATER!!! --------

// Main
extern "C" void app_main(void) 
{
    // This tells the C++ compiler: 
    // "Don't mangle this name. Keep it exactly as it is (C-style)."

    initArduino();

    // 2. Call setup and capture the result
    esp_err_t err = gimbal.setup();

    // 3. Evaluate the result
    if (err != ESP_OK) {
        // Use the built-in error-to-string helper for debugging
        printf("Gimbal Setup FAILED: %s (0x%X)\n", esp_err_to_name(err), err);
        
        // Safety: Don't start the PID loop if the IMU or Encoders are offline
        // abort();
    }
    else {
        printf("%d\n", err);
    }
    
    // Instead of running on a loop, we use a timer interrupt
    // to trigger the update function at a fixed frequency

    // Have a timer interrupt trigger a flag and nothing else during the ISR
    // if the flag is triggered, then run the update function in the main loop.
    // if (flag) {
    //     gimbal.update();
    // }

    // // Initialise Gimbal
    // while (true) {
    //     gimbal.update();
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }
}