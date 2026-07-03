#include "LSM6DSV16X_IMU.h"

#include <cstring>

static const char* TAG = "LSM6DSV16X_IMU";

// Helper function to decode IEEE 754 Half-Precision Float
float half_to_float(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x00000001;
    uint32_t exp  = (h >> 10) & 0x0000001F;
    uint32_t frac =  h        & 0x000003FF;

    // Handle subnormal numbers as 0 for IMU gimbal purposes
    if (exp == 0) return 0.0f; 

    // Convert to 32-bit float: Re-bias the exponent (+112) and shift fraction
    uint32_t f_bits = (sign << 31) | ((exp + 112) << 23) | (frac << 13);
    
    float f;
    memcpy(&f, &f_bits, 4); // Safe memory copy to float
    return f;
}

LSM6DSV16X_IMU::LSM6DSV16X_IMU(int cs_pin) : _cs_pin(cs_pin), _spi_handle(nullptr) {}

// --- The Single Byte Function ---
esp_err_t LSM6DSV16X_IMU::write_reg(uint8_t reg_address, uint8_t data) {
    // This simply packages the single byte and hands it to the burst function
    return write_reg(reg_address, &data, 1);
}

// --- The Multi-Byte Burst Function ---
esp_err_t LSM6DSV16X_IMU::write_reg(uint8_t reg_address, uint8_t *data, size_t len) {
    const size_t MAX_BUF_SIZE = 32; 
    if (len + 1 > MAX_BUF_SIZE) return ESP_ERR_NO_MEM; 

    uint8_t tx_buf[MAX_BUF_SIZE] = {0};

    // 1. First byte: WRITE bit (0) + 7-bit Address
    tx_buf[0] = reg_address & 0x7F;

    // 2. Copy the user's data array
    memcpy(&tx_buf[1], data, len);

    // 3. Configure the transaction
    spi_transaction_t t = {};
    t.length = 8 * (len + 1);     
    t.tx_buffer = tx_buf;
    t.rx_buffer = nullptr;        

    // 4. Execute the transfer
    return spi_device_transmit(_spi_handle, &t);
}

esp_err_t LSM6DSV16X_IMU::read_reg(uint8_t reg_address, uint8_t *receive, size_t len) {
    // 1. Create fast local buffers for Full Duplex transmission.
    // 32 bytes is plenty for a 7-byte quaternion or a 14-byte raw sensor burst.
    const size_t MAX_BUF_SIZE = 32; 
    if (len + 1 > MAX_BUF_SIZE) return ESP_ERR_NO_MEM; // Guard rail

    // Initialize all array elements to 0x00 (These become our 'dummy' bytes)
    uint8_t tx_buf[MAX_BUF_SIZE] = {0}; 
    uint8_t rx_buf[MAX_BUF_SIZE] = {0};

    // 2. Set the first byte: READ bit (1) + 7-bit Address
    tx_buf[0] = reg_address | 0x80;

    // 3. Configure the transaction
    spi_transaction_t t = {};
    t.length = 8 * (len + 1);     // Total bits: Address byte + Data bytes
    t.rxlength = 8 * (len + 1);   // Must exactly match t.length in Full Duplex
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    // 4. Execute the transfer
    esp_err_t ret = spi_device_transmit(_spi_handle, &t);

    if (ret == ESP_OK) {
        // 5. Extract the valid data.
        // rx_buf[0] contains garbage received while sending the address.
        // The real data starts at rx_buf[1], so we copy it to the user's pointer.
        memcpy(receive, &rx_buf[1], len);
    }

    return ret;
}

esp_err_t LSM6DSV16X_IMU::enable_SFLP()
{
    esp_err_t ret;

    // 1. Enable accelerometer register at 480Hz
    ret = write_reg(Regs::CTRL1, 0b00001000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Set accelerometer data rate 480Hz!");
    }
    else {
        ESP_LOGE(TAG, "Failed to set accelerometer data rate!");
        return ret;
    }

    // 2. Enable gyroscope register at 480Hz
    ret = write_reg(Regs::CTRL2, 0b00001000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Set gyroscope data rate 480Hz!");
    }
    else {
        ESP_LOGE(TAG, "Failed to set gyroscope data rate!");
        return ret;
    }

    // 3. Point to embedded functions memory bank
    ret = write_reg(Regs::FUNC_CFG_ACCESS, 0b10000000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled embedded functions registers!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable embedded functions registers!");
        return ret;
    }

    // 4. Enable SFLP game vector
    ret = write_reg(Regs::EMB_FUNC_EN_A, 0b00000010);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled SFLP game vector!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable SFLP game vector!");
        return ret;
    }

    // 5. Configure SFLP data rate to 480Hz
    ret = write_reg(Regs::SFLP_ODR, 0b01101011);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully set SFLP data rate to 480Hz!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable SFLP game vector!");
        return ret;
    }

    // 6. Enable batching of SFLP data to the FIFO
    ret = write_reg(Regs::EMB_FUNC_FIFO_EN_A, 0b00000010);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled batching of SFLP to the FIFO!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable batching of SFLP to the FIFO!");
        return ret;
    }

    // 7. Point back to default memory bank so we can access the FIFO registers
    ret = write_reg(Regs::FUNC_CFG_ACCESS, 0b00000000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully disabled embedded functions registers!");
    }
    else {
        ESP_LOGE(TAG, "Failed to disable embedded functions registers!");
        return ret;
    }

    // 8. Enable continuous mode for the FIFO
    ret = write_reg(Regs::FIFO_CTRL4, 0b00000110);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled continuous mode for the FIFO!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable continuous mode for the FIFO!");
        return ret;
    }

    // Check FIFO registers for SFLP game rotation vector (returns 0x13)
    uint8_t FIFO_tag;
    ret = read_reg(Regs::FIFO_DATA_OUT_TAG, &FIFO_tag);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully requested the FIFO tag!");
        uint8_t sensor_tag = (FIFO_tag & 0b11111000) >> 3;

        if (sensor_tag == 0x13) {
            ESP_LOGI(TAG, "FIFO tag detected! ID: 0x%02X", sensor_tag);
        }
        else {
            ESP_LOGE(TAG, "FIFO tag mismatch! Expected 0x13, got 0x%02X", sensor_tag);
            return ESP_ERR_NOT_FOUND;
        }
    }
    else {
        ESP_LOGE(TAG, "Failed to request the FIFO tag!");
        return ret;
    }

    return ESP_OK;
}

esp_err_t LSM6DSV16X_IMU::begin(spi_host_device_t spi_host)
{
    printf("---- Starting IMU setup sequence ----\n");
    // Configure device (based on ESP32 SPI library)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1e6;               // 10 MHz
    devcfg.mode = 3;
    devcfg.spics_io_num = _cs_pin;
    devcfg.queue_size = 7;

    // Adding IMU to the SPI bus
    esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &_spi_handle);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully added to the SPI bus.");
    }
    else {
        ESP_LOGE(TAG, "Could not be added to the SPI bus!");
        return ret;
    }
    
    // Read the IMU's ID
    uint8_t IMU_id;
    ret = read_reg(Regs::WHO_AM_I, &IMU_id);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully transmitted device ID message!");
    }
    else {
        ESP_LOGE(TAG, "Failed to request the device ID!");
        return ret;
    }

    // Check the value of the ID
    if (IMU_id == 0x70) { 
        ESP_LOGI(TAG, "LSM6DSV16X detected! ID: 0x%02X", IMU_id);
    }
    else {
        ESP_LOGE(TAG, "Device mismatch! Expected 0x70, got 0x%02X", IMU_id);
        return ESP_ERR_NOT_FOUND;
    }

    // Enable quaternion game vector
    ret = enable_SFLP();
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully completed SFLP game vector setup!");
    }
    else {
        ESP_LOGE(TAG, "Failed to setup SFLP game vector!");
        return ret;
    }

    // If passed all checks, return OK status
    return ESP_OK;
}

esp_err_t LSM6DSV16X_IMU::update_quaternion(Quaternion *q)
{
    // Read FIFO registers for quaternion data
    uint8_t raw_quaternion[7];
    esp_err_t ret = read_reg(Regs::FIFO_DATA_OUT_TAG, raw_quaternion, sizeof(raw_quaternion));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensor read failed!");
        return ret;
    }
    
    uint8_t sensor_tag = (raw_quaternion[0] & 0b11111000) >> 3;

    // If the SFLP tag isn't for a quaternion, then return an error
    if (sensor_tag != 0x13) {
        ESP_LOGE(TAG, "Wrong sensor tag detected! Detected 0x%x when it should be 0x13!", sensor_tag);
        return ESP_FAIL;
    }

// 1. Reconstruct the 16-bit words (Little Endian)
    uint16_t raw_x = (raw_quaternion[2] << 8) | raw_quaternion[1];
    uint16_t raw_y = (raw_quaternion[4] << 8) | raw_quaternion[3];
    uint16_t raw_z = (raw_quaternion[6] << 8) | raw_quaternion[5];

    // 2. Decode the half-precision floats
    q->x = half_to_float(raw_x);
    q->y = half_to_float(raw_y);
    q->z = half_to_float(raw_z);

    // 3. Calculate the W component
    float mag_xyz_sq = q->x * q->x + q->y * q->y + q->z * q->z;
    q->w = sqrtf(fmaxf(0.0f, 1.0f - mag_xyz_sq));

    // Optional: Print the real values to check your success!
    ESP_LOGI(TAG, "Quaternion: w:%.3f x:%.3f y:%.3f z:%.3f", q->w, q->x, q->y, q->z);

    return ESP_OK;
}


    // check the SFLP game vector register to see if the value was changed
    // uint8_t sflp_value;
    // ret = read_reg(Regs::EMB_FUNC_EN_A, &sflp_value);
    // if (ret == ESP_OK) {
    //     ESP_LOGE(TAG, "Successfully read the SFLP register!");

    //     if (sflp_value == 0b00000010) {
    //         ESP_LOGI(TAG, "SFLP enabled! Register: 0x%02X", sflp_value);
    //     }
    //     else {
    //         ESP_LOGE(TAG, "SFLP not enabled! Register: 0x%02X", sflp_value);
    //         return ESP_ERR_NOT_FOUND;
    //     }
    // }
    // else {
    //     ESP_LOGE(TAG, "Failed to disable embedded functions registers!");
    //     return ret;
    // }