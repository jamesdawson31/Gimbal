#ifndef GIMBAL_H
#define GIMBAL_H

#include "Device.h"
#include "SPIDevice.h"
#include "IMU.h"
#include "Encoder.h"

class Gimbal 
{
    public:
        Gimbal(IMU* imu, Encoder* yaw, Encoder* pitch, Encoder* roll);

        esp_err_t Gimbal::begin() {
            // Check device status 
        }

    private:
        // Array of pointers to devices
        Device* _components[4];

        // Individual pointers to specific devices
        IMU* _imu;
        Encoder* _yaw_enc;
        Encoder* _pitch_enc;
        Encoder* _roll_enc;
        // BMS* _bms;
        // ControlInterface* _control_interface;        // for later implementation of buttons, screen, etc.
};

#endif // GIMBAL_H