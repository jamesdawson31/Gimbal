#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "SPIDevice.h"
#include "Encoder.h"

Encoder::Encoder(int cs_pin) : _cs_pin(cs_pin) {}

esp_err_t Encoder::begin(spi_host_device_t host) {
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10MHz max [cite: 355]
    devcfg.mode = 1;                         // Correct mode for AS5048A 
    devcfg.spics_io_num = _cs_pin;
    devcfg.queue_size = 7;

    esp_err_t ret = spi_bus_add_device(host, &devcfg, &_spi_handle);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10)); // Vital startup delay 
    return ESP_OK;
}