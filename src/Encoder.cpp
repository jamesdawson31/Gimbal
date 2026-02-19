#include "Device.h"
#include "SPIDevice.h"
#include "Encoder.h"

Encoder::Encoder(int cs_pin) : _cs_pin(cs_pin) {}

esp_err_t Encoder::begin() {
    
}