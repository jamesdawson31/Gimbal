#include "Gimbal.h"

static const char* TAG = "GIMBAL";

Gimbal::Gimbal(SPIBus* spi_bus, IMU* imu, Encoder* yaw, Encoder* pitch, Encoder* roll) 
    : _spi_bus(spi_bus), _imu(imu), _yaw_enc(yaw), _pitch_enc(pitch), _roll_enc(roll) 
{
    // Fill the components array
    _spi_components[0] = imu;
    _spi_components[1] = yaw;
    _spi_components[2] = pitch;
    _spi_components[3] = roll;
}

esp_err_t Gimbal::setup() 
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n=== System Booting ===\n");
    
    printf("gimbal sequence\n");
    ESP_LOGI(TAG, "Starting Gimbal setup sequence...\n");
    // Check device status 

    // Initialise SPIBus
    if (_spi_bus->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus failed to initialise!");
        return ESP_FAIL;
    }
    else {
        ESP_LOGE(TAG, "SPI Bus successfully initialised!");
    }
    spi_host_device_t spi_host = _spi_bus->get_host();

    // Initialise IMU
    if (_imu->begin(spi_host) != ESP_OK) {
        ESP_LOGE(TAG, "IMU failed to initialise!");
        return ESP_FAIL;
    }
    else {
        ESP_LOGE(TAG, "IMU successfully initialised!");
    }
    

    // std::cout << "Worked!" << std::endl;
    
    // Initialise YAW Encoder
    // _yaw_enc->begin(spi_host);

    // Initialise PITCH Encoder
    // _pitch_enc->begin(spi_host);

    // Initialise ROLL Encoder
    // _roll_enc->begin(spi_host);


    // Setup interrupts etc.

    return ESP_OK;          // placeholder!
}

// void Gimbal::update()
// {
//     // Get sensor updates and run control loop here
//     // Implement control 
// }