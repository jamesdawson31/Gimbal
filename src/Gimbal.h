#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include "SPIBus.h"
#include "SPIDevice.h"
#include "LSM6DSV16X_IMU.h"
#include "AS5048AEncoder.h"

class Gimbal 
{
    public:
        Gimbal(SPIBus* spi_bus, LSM6DSV16X_IMU* imu, AS5048AEncoder* yaw, AS5048AEncoder* pitch, AS5048AEncoder* roll);

        esp_err_t setup();
        esp_err_t update();      // change datatype later

    private:
        // Pointer to the SPI bus
        SPIBus* _spi_bus;

        // Array of pointers to devices
        SPIDevice* _spi_components[4];

        // Individual pointers to specific devices
        LSM6DSV16X_IMU* _imu;
        AS5048AEncoder* _yaw_enc;
        AS5048AEncoder* _pitch_enc;
        AS5048AEncoder* _roll_enc;
        // BMS* _bms;
        // ControlInterface* _control_interface;        // for later implementation of buttons, screen, etc.

        // Quaternion
        Quaternion _q;
};

#endif // GIMBAL_H