#ifndef SPIBUS_H
#define SPIBUS_H

#include "driver/spi_master.h"
#include "esp_log.h"

class SPIBus {
    public:
        // Constructor
        SPIBus(spi_host_device_t spi_bus, int mosi, int miso, int sclk);

        // Method to initialise the shared wires (MISO, MOSI, SCLK)
        esp_err_t initialise();

        // Method to return which SPI bus we are using on the ESP32-S3
        spi_host_device_t get_host();

        

    private:
        // Specific ESP32 SPI bus
        spi_host_device_t _spi_host;
    
        // Pins that belong to an SPI bus
        int _mosi;
        int _miso;
        int _sclk;

        // Already initialised check variable/flag
        static bool _is_initialised;

};

#endif // SPIBUS_H