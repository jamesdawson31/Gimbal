#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include "SPIBus.h"
#include "LSM6DSV16X_IMU.h"
#include "AS5048AEncoder.h"

class SpeedControl 
{
    public:
        SpeedControl(SPIBus* spi_bus, AS5048AEncoder* encoder);

        esp_err_t setup();
        esp_err_t update();      // change datatype later

    private:
        // Pointer to the SPI bus
        SPIBus* _spi_bus;

        // Pointer to the encoder
        AS5048AEncoder* _encoder;
};

#endif // SPEEDCONTROL_H