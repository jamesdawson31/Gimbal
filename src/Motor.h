#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    public:
        Motor(int pin_pwm_1, int pin_pwm_2, int pin_pwm_3);
        float getPosition();
        float getVelocity();

    private:
        int _pin_pwm_1;
        int _pin_pwm_2;
        int _pin_pwm_3;
        float _position;
        float _velocity;
};


#endif // MOTOR_H