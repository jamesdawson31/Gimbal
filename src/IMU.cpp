#include "IMU.h"

static const char* TAG = "IMU";

IMU::IMU(int cs_pin) : _cs_pin(cs_pin), _spi_handle(nullptr) {}

esp_err_t IMU::write_reg(uint8_t reg_address, uint8_t data)
{
    uint8_t tx_data[2] = {reg_address, data};
    
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = nullptr;      // no need to receive data for writing

    return spi_device_transmit(_spi_handle, &t);
}

esp_err_t IMU::read_reg(uint8_t reg_address, uint8_t *receive)
{
    // tx_data: register address sent to the IMU
    // rx_data: buffer for data to be received back from the IMU
    uint8_t tx_data[2] = {(uint8_t)(reg_address | Regs::READ_BIT), 0x00};
    uint8_t rx_data[2] = {0, 0};

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;              // pass by pointer directly edits 

    esp_err_t ret = spi_device_transmit(_spi_handle, &t);

    if (ret == ESP_OK) {
        *receive = rx_data[1];
    }

    return ret;
}

esp_err_t IMU::enable_SFLP()
{
    esp_err_t ret;
    // 1. Point to embedded functions memory bank
    ret = write_reg(Regs::FUNC_CFG_ACCESS, 0b10000000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled embedded functions registers!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable embedded functions registers!");
        return ret;
    }

    // 2. Enable SFLP game vector or whatever
    ret = write_reg(Regs::EMB_FUNC_EN_A, 0b00000010);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully enabled SFLP game vector!");
    }
    else {
        ESP_LOGE(TAG, "Failed to enable SFLP game vector!");
        return ret;
    }

    // 3. Point back to default memory bank so we can access the FIFO registers
    ret = write_reg(Regs::FUNC_CFG_ACCESS, 0b00000000);
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully disabled embedded functions registers!");
    }
    else {
        ESP_LOGE(TAG, "Failed to disable embedded functions registers!");
        return ret;
    }

    // Check FIFO registers for SFLP game rotation vector (returns 0x13)
    uint8_t FIFO_tag;
    ret = read_reg(Regs::FIFO_DATA_OUT_TAG, &FIFO_tag);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "Successfully requested the FIFO tag!");
    }
    else {
        ESP_LOGE(TAG, "Failed to request the FIFO tag!");
        return ret;
    }

    if (FIFO_tag == 0x13) {
        ESP_LOGI(TAG, "FIFO tag detected! ID: 0x%02X", FIFO_tag);
    }
    else {
        ESP_LOGE(TAG, "FIFO tag mismatch! Expected 0x13, got 0x%02X", FIFO_tag);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t IMU::begin(spi_host_device_t spi_host)
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

