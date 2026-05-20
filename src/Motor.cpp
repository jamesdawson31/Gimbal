#include "Motor.h"

Motor::Motor(int pin_pwm_1, int pin_pwm_2, int pin_pwm_3) 
    : _pin_pwm_1(pin_pwm_1), _pin_pwm_2(pin_pwm_2), _pin_pwm_3(pin_pwm_3), _position(0.0), _velocity(0.0) 
{
    
}