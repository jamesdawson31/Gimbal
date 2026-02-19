#include "Gimbal.h"

Gimbal::Gimbal(IMU* imu, Encoder* yaw, Encoder* pitch, Encoder* roll) 
    : _imu(imu), _yaw_enc(yaw), _pitch_enc(pitch), _roll_enc(roll) 
{
    // Fill the components array
    _components[0] = imu;
    _components[1] = yaw;
    _components[2] = pitch;
    _components[3] = roll;
}

esp_err_t Gimbal::setup() 
{
    // Check device status 

    // Initialise SPIBus
    _spi_bus->initialise();

    // Initialise IMU
    _imu->begin();

    // Initialise YAW Encoder


    // Initialise PITCH Encoder


    // Initialise ROLL Encoder



    // Setup interrupts etc.
}

void Gimbal::update()
{
    // Get sensor updates and run control loop here
    // Implement control 
}