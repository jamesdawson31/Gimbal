#include "SPIBus.h"

#include "driver/spi_master.h"


SPIBus::SPIBus(int mosi, int miso, int sclk) : _mosi(mosi, )

spi_host_device_t SPIBus::get_host() {
    return SPI2_HOST;
}

esp_err_t SPIBus::initialise(int mosi, int miso, int sclk) {
    // Create bus configuration struct object
    spi_bus_config_t buscfg = {};       // {} initialises with default values

    // Configure the bus (buscfg) MOSI, MISO, SCLK, etc
    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
    buscfg.sclk_io_num = sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4092;

    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}