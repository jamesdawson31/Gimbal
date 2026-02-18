// Standard libraries
#include <stdlib.h>

// Custom system libraries
#include "Gimbal.h"
#include "Motor.h"
#include "Encoder.h"
#include "IMU.h"
#include "SPIBus.h"

// Pin definitions
#define PIN_SPI_MOSI            35
#define PIN_SPI_MISO            37
#define PIN_SPI_SCLK            36
#define PIN_SPI_IMU_CS          34
#define PIN_SPI_ENC_1_CS        9
#define PIN_SPI_ENC_2_CS        8
#define PIN_SPI_ENC_3_CS        7
// Add BMS and power management stuff later
// #define 


// Main
extern "C" void app_main(void) {
    // This tells the C++ compiler: 
    // "Don't mangle this name. Keep it exactly as it is (C-style)."

    // Initialise the SPI Bus
    SPIBus::initialise(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK);

    

}