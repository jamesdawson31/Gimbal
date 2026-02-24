#ifndef IMU_H
#define IMU_H

#include "driver/spi_master.h"
#include "SPIDevice.h"

// bit 0: READ bit. The value is 1.
// bit 1-7: address AD(6:0). This is the address field of the indexed register.
// bit 8-15: data DO(7:0) (read mode). This is the data that is read from the device (MSb first).
// bit 16-...: data DO(...-8). Further data in multiple byte reads.

// The memory in the IMU is partitioned into "banks". 
// The value of the FUNC_CFG_ACCESS register determines which bank is active. 
// 

struct Quaternion {
    float x, y, z, w;
};

class IMU : public SPIDevice 
{
    public:
        // Public attributes
        IMU(int cs_pin);
        esp_err_t begin(spi_host_device_t spi_bus) override;
        // esp_err_t update(Quaternion &q);
        
    private:
        // Private attributes
        int _cs_pin;
        spi_device_handle_t _spi_handle;        // identifier for each SPI device

        // Setup functions
        esp_err_t write_reg(uint8_t reg_address, uint8_t data);
        esp_err_t read_reg(uint8_t reg_address, uint8_t *receive);
        esp_err_t enable_SFLP();

        // Registers
        struct Regs {
            static constexpr uint8_t READ_BIT               = 0x80; // 
            static constexpr uint8_t WHO_AM_I               = 0x0F; //
            static constexpr uint8_t STATUS_REG_OIS         = 0x1E; // 
            static constexpr uint8_t OUT_TEMP_L             = 0x20; // 
            static constexpr uint8_t OUT_TEMP_H             = 0x21; // 
            
            // Gyroscope Output (OIS)
            static constexpr uint8_t OUTX_L_G_OIS           = 0x22; // 
            static constexpr uint8_t OUTX_H_G_OIS           = 0x23; // 
            static constexpr uint8_t OUTY_L_G_OIS           = 0x24; // 
            static constexpr uint8_t OUTY_H_G_OIS           = 0x25; // 
            static constexpr uint8_t OUTZ_L_G_OIS           = 0x26; // 
            static constexpr uint8_t OUTZ_H_G_OIS           = 0x27; // 
            
            // Accelerometer Output (OIS)
            static constexpr uint8_t OUTX_L_A_OIS           = 0x28; // 
            static constexpr uint8_t OUTX_H_A_OIS           = 0x29; // 
            static constexpr uint8_t OUTY_L_A_OIS           = 0x2A; // 
            static constexpr uint8_t OUTY_H_A_OIS           = 0x2B; // 
            static constexpr uint8_t OUTZ_L_A_OIS           = 0x2C; // 
            static constexpr uint8_t OUTZ_H_A_OIS           = 0x2D; // 

            // Control Registers
            static constexpr uint8_t HANDSHAKE_CTRL         = 0x6E; // 
            static constexpr uint8_t INT_OIS                = 0x6F; // 
            static constexpr uint8_t CTRL1_OIS              = 0x70; // 
            static constexpr uint8_t CTRL2_OIS              = 0x71; // 
            static constexpr uint8_t CTRL3_OIS              = 0x72; // 

            // Embedded Functions Registers
            static constexpr uint8_t FUNC_CFG_ACCESS        = 0x01; // used to enable the embedded functions register
            static constexpr uint8_t EMB_FUNC_EN_A          = 0x04;

            // Quaternion Output Registers
            static constexpr uint8_t FIFO_DATA_OUT_TAG      = 0x78; // returns 0x13 if SFLP game vector is enabled
            static constexpr uint8_t FIFO_DATA_OUT_X_L      = 0x79;
            static constexpr uint8_t FIFO_DATA_OUT_X_H      = 0x7A;
            static constexpr uint8_t FIFO_DATA_OUT_Y_L      = 0x7B;
            static constexpr uint8_t FIFO_DATA_OUT_Y_H      = 0x7C;
            static constexpr uint8_t FIFO_DATA_OUT_Z_L      = 0x7D;
            static constexpr uint8_t FIFO_DATA_OUT_Z_H      = 0x7E;
        };
};

#endif // IMU_H