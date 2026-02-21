#ifndef IMU_H
#define IMU_H

#include "driver/spi_master.h"
#include "SPIDevice.h"

class IMU : public SPIDevice 
{
    public:
        // Public attributes
        IMU(int cs_pin);
        esp_err_t begin(spi_host_device_t spi_bus) override;
        esp_err_t IMU::update(Quaternion &q);
        
    private:
        // Private attributes
        int _cs_pin;
        spi_device_handle_t _spi_handle;        // identifier for each SPI device

        struct Quaternion {
            float x, y, z, w;
        };
};

#endif // IMU_H