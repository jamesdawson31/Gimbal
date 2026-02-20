#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include "SPIBus.h"
#include "SPIDevice.h"
#include "IMU.h"
#include "Encoder.h"

class Gimbal 
{
    public:
        Gimbal(SPIBus* spi_bus, IMU* imu, Encoder* yaw, Encoder* pitch, Encoder* roll);

        esp_err_t setup();
        // void Gimbal::update();      // change datatype later

    private:
        // 

        // Array of pointers to devices
        SPIDevice* _spi_components[4];

        // Pointer to the SPI bus
        SPIBus* _spi_bus;

        // Individual pointers to specific devices
        IMU* _imu;
        Encoder* _yaw_enc;
        Encoder* _pitch_enc;
        Encoder* _roll_enc;
        // BMS* _bms;
        // ControlInterface* _control_interface;        // for later implementation of buttons, screen, etc.
};

#endif // GIMBAL_H