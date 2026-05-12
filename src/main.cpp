// Standard libraries
#include <stdlib.h>
#include <stdio.h>

// ESP32 libraries
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Arduino libraries
#include <Arduino.h>

// Custom system libraries
#include "Gimbal.h"
#include "Motor.h"
#include "Encoder.h"
#include "IMU.h"
#include "SPIBus.h"

// Pin definitions
#define PIN_SPI_MOSI                35
#define PIN_SPI_MISO                37
#define PIN_SPI_SCLK                36
#define PIN_SPI_IMU_CS              34
#define PIN_SPI_ENC_YAW_CS          9
#define PIN_SPI_ENC_PITCH_CS        8
#define PIN_SPI_ENC_ROLL_CS         7
// Add BMS and power management stuff later
// #define 

// Initialisation of classes
SPIBus  spi_bus_2(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK);
IMU     imu(PIN_SPI_IMU_CS);
Encoder yaw_enc(PIN_SPI_ENC_YAW_CS);
Encoder pitch_enc(PIN_SPI_ENC_PITCH_CS);
Encoder roll_enc(PIN_SPI_ENC_ROLL_CS);
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
    
    // Initialise Gimbal
    while (true) {
        gimbal.update();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}