#include "SPIBus.h"

#include "driver/spi_master.h"

bool SPIBus::_is_initialised = false;

SPIBus::SPIBus(spi_host_device_t spi_host, int mosi, int miso, int sclk) 
    : _spi_host(spi_host), _mosi(mosi), _miso(miso), _sclk(sclk) 
{
    if (_is_initialised) {
        // Throw an error if the bus is already initialised
        ESP_LOGE("SPI_BUS", "Error: SPI Bus instance already exists!");

        // Abort the program
        abort();
    }

    _is_initialised = true;
}

spi_host_device_t SPIBus::get_host() {
    return _spi_host;
}

esp_err_t SPIBus::initialise() {
    // Create bus configuration struct object
    spi_bus_config_t buscfg = {};       // {} initialises with default values

    // Configure the bus (buscfg) MOSI, MISO, SCLK, etc
    buscfg.mosi_io_num = _mosi;
    buscfg.miso_io_num = _miso;
    buscfg.sclk_io_num = _sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4092;

    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}