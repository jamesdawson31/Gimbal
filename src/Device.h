#ifndef DEVICE_H
#define DEVICE_H

#include "esp_err.h"

class Device 
{
    public:

    protected:
        virtual esp_err_t begin() = 0;
        // virtual esp_err_t update() = 0;

};

#endif // DEVICE_H