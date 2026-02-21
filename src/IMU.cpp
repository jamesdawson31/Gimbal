#include "IMU.h"

// LSM6DSV16X Register Map
#define REG_EMB_FUNC_EN_A
// #define REG_


IMU::IMU(int cs_pin) : _cs_pin(cs_pin), _spi_handle(nullptr) {}

esp_err_t IMU::begin(spi_host_device_t spi_host)
{
    // Configure device (based on ESP32 SPI library)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1e6;               // 10 MHz
    devcfg.mode = 3;
    devcfg.spics_io_num = _cs_pin;
    devcfg.queue_size = 7;

    return spi_bus_add_device(spi_host, &devcfg, &_spi_handle);
}

