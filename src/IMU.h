#ifndef IMU_H
#define IMU_H

#include "driver/spi_master.h"
#include "Device.h"
#include "SPIDevice.h"

class IMU {
    public:
        // Public attributes
        IMU(int cs_pin);

    private:
        // Private attributes
        int _cs_pin;
        spi_device_handle_t _spi_handle;        // identifier for each SPI device
};

#endif // IMU_H