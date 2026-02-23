#ifndef IMU_H
#define IMU_H

#include "driver/spi_master.h"
#include "SPIDevice.h"

// bit 0: READ bit. The value is 1.
// bit 1-7: address AD(6:0). This is the address field of the indexed register.
// bit 8-15: data DO(7:0) (read mode). This is the data that is read from the device (MSb first).
// bit 16-...: data DO(...-8). Further data in multiple byte reads.

struct Quaternion {
    float x, y, z, w;
};

class IMU : public SPIDevice 
{
    public:
        // Public attributes
        IMU(int cs_pin);
        esp_err_t begin(spi_host_device_t spi_bus) override;
        esp_err_t IMU::update(Quaternion &q);
        
    private:
        // Private attributes
        int _cs_pin;
        spi_device_handle_t _spi_handle;        // identifier for each SPI device

        // Registers
        struct Regs {
            static constexpr uint8_t READ_BIT            = 0x80; // 
            static constexpr uint8_t WHO_AM_I            = 0x0F; // [cite: 518]
            static constexpr uint8_t STATUS_REG_OIS      = 0x1E; // 
            static constexpr uint8_t OUT_TEMP_L          = 0x20; // 
            static constexpr uint8_t OUT_TEMP_H          = 0x21; // 
            
            // Gyroscope Output (OIS)
            static constexpr uint8_t OUTX_L_G_OIS       = 0x22; // 
            static constexpr uint8_t OUTX_H_G_OIS       = 0x23; // 
            static constexpr uint8_t OUTY_L_G_OIS       = 0x24; // 
            static constexpr uint8_t OUTY_H_G_OIS       = 0x25; // 
            static constexpr uint8_t OUTZ_L_G_OIS       = 0x26; // 
            static constexpr uint8_t OUTZ_H_G_OIS       = 0x27; // 
            
            // Accelerometer Output (OIS)
            static constexpr uint8_t OUTX_L_A_OIS       = 0x28; // 
            static constexpr uint8_t OUTX_H_A_OIS       = 0x29; // 
            static constexpr uint8_t OUTY_L_A_OIS       = 0x2A; // 
            static constexpr uint8_t OUTY_H_A_OIS       = 0x2B; // 
            static constexpr uint8_t OUTZ_L_A_OIS       = 0x2C; // 
            static constexpr uint8_t OUTZ_H_A_OIS       = 0x2D; // 

            // Control Registers
            static constexpr uint8_t HANDSHAKE_CTRL      = 0x6E; // 
            static constexpr uint8_t INT_OIS             = 0x6F; // 
            static constexpr uint8_t CTRL1_OIS           = 0x70; // 
            static constexpr uint8_t CTRL2_OIS           = 0x71; // 
            static constexpr uint8_t CTRL3_OIS           = 0x72; // 
        };
};

#endif // IMU_H