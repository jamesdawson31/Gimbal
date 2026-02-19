#ifndef DEVICE_H
#define DEVICE_H

class Device 
{
    protected:
        virtual esp_err_t begin() = 0;

};

#endif // DEVICE_H