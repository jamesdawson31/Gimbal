#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SpeedControl.h"

static const char* TAG = "SPEED_CONTROL";

SpeedControl::SpeedControl(SPIBus* spi_bus, AS5048AEncoder* encoder) 
    : _spi_bus(spi_bus), _encoder(encoder) {}

esp_err_t SpeedControl::setup() 
{
    // Initial delay of 1s to startup
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n\n======== System Booting ========\n");

    // Initialise SPIBus
    if (_spi_bus->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus failed to initialise!");
        return ESP_FAIL;
    }
    else {
        ESP_LOGE(TAG, "SPI Bus successfully initialised!");
    }
    spi_host_device_t spi_host = _spi_bus->get_host();


    return ESP_OK;          // placeholder!
}