#include "Gimbal.h"

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
    // Check device status 

    // Initialise SPIBus
    _spi_bus->initialise();

    // Initialise IMU
    _imu->begin(_spi_bus->get_host());

    std::cout << "Worked!" << std::endl;
    // Initialise YAW Encoder


    // Initialise PITCH Encoder


    // Initialise ROLL Encoder



    // Setup interrupts etc.

    return ESP_OK;
}

// void Gimbal::update()
// {
//     // Get sensor updates and run control loop here
//     // Implement control 
// }