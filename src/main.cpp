// Standard libraries
#include <stdlib.h>

// ESP32 libraries
#include "esp_err.h"

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

// Static allocation
SPIBus  spi_bus(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK);
IMU     imu(PIN_SPI_IMU_CS);
Encoder yaw_enc(PIN_SPI_ENC_YAW_CS);
Encoder pitch_enc(PIN_SPI_ENC_PITCH_CS);
Encoder roll_enc(PIN_SPI_ENC_ROLL_CS);

// -------- CAN OPTIMISE INTEGER SIZES LATER!!! --------

// Main
extern "C" void app_main(void) {
    // This tells the C++ compiler: 
    // "Don't mangle this name. Keep it exactly as it is (C-style)."

    // 
    Gimbal gimbal();

    // Initialise Gimbal
    

    

}