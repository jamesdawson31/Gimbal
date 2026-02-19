#include "IMU.h"

// LSM6DSV16X Register Map
#define REG_EMB_FUNC_EN_A
#define REG_


IMU::IMU(int cs_pin) : _cs_pin(cs_pin), _spi_handle(nullptr) {}

esp_err_t IMU::begin()
{
    // Configure device (based on ESP32 SPI library)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1e6                // 10 MHz
}