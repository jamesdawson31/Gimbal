#ifndef ENCODER_H
#define ENCODER_H

#include "SPIDevice.h"
#include "esp_log.h"

class Encoder : public SPIDevice 
{
    public:
        Encoder(int cs_pin);

        esp_err_t begin(spi_host_device_t spi_host) override;

    private:
        int _cs_pin;
        spi_device_handle_t _spi_handle;        // identifier for each SPI device
};

#endif // ENCODER_H