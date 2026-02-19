#ifndef ENCODER_H
#define ENCODER_H

#include "Device.h"
#include "SPIDevice.h"

class Encoder : public SPIDevice 
{
    public:
        Encoder(int cs_pin);

        esp_err_t begin();

    private:
        int _cs_pin;
};

#endif // ENCODER_H