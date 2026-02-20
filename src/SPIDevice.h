#ifndef SPIDEVICE_H
#define SPIDEVICE_H


#include "driver/spi_master.h"
#include "esp_err.h"

class SPIDevice 
{
    public:

    protected:
        virtual esp_err_t begin(spi_host_device_t spi_host) = 0;
        // virtual esp_err_t update() = 0;

};

#endif // SPIDEVICE_H