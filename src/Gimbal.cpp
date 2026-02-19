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