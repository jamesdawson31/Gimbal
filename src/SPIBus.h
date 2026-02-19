#ifndef SPIBUS_H
#define SPIBUS_H

#include "driver/spi_master.h"

class SPIBus {
    public:
        // Constructor
        SPIBus(int mosi, int miso, int sclk);

        // Method to initialise the shared wires (MISO, MOSI, SCLK)
        static esp_err_t initialise();

        // Method to return which SPI bus we are using on the ESP32-S3
        static spi_host_device_t get_host();

        

    private:
        int _mosi;
        int _miso;
        int _sclk;
};

#endif // SPIBUS_H