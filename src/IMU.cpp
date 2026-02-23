#include "IMU.h"

// LSM6DSV16X Register Map
#define REG_EMB_FUNC_EN_A

static const char* TAG = "IMU";

IMU::IMU(int cs_pin) : _cs_pin(cs_pin), _spi_handle(nullptr) {}

esp_err_t IMU::write_reg(uint8_t reg_address, uint8_t data)
{
    uint8_t transmit[2] = {reg_address, data};
    
    spi_transaction_t t = {};
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &transmit;
    t.rx_buffer = nullptr;      // no need to receive data for writing

    return spi_device_transmit(_spi_handle, &t);
}

esp_err_t IMU::read_reg(uint8_t reg_address, uint8_t *receive)
{
    spi_transaction_t t = {};
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &reg_address;
    t.rx_buffer = receive;              // pass by pointer directly edits 

    return spi_device_transmit(_spi_handle, &t);
}

esp_err_t IMU::enable_SFLP()
{
    // 1. point to embedded functions memory bank
    // 2. enable SFLP game vector or whatever
    // 3. point back to default memory bank so we can access the FIFO
}

esp_err_t IMU::begin(spi_host_device_t spi_host)
{
    // Configure device (based on ESP32 SPI library)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1e6;               // 10 MHz
    devcfg.mode = 3;
    devcfg.spics_io_num = _cs_pin;
    devcfg.queue_size = 7;

    esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &_spi_handle);
    // Exit setup if something fails
    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "IMU successfully added to the SPI bus.");
    }
    else {
        ESP_LOGE(TAG, "IMU could not be added to the SPI bus!");
        return ret;
    }

    // Prepare WHO_AM_I request
    uint8_t transmit = Regs::WHO_AM_I | Regs::READ_BIT;
    uint8_t receive = 0;

    spi_transaction_t t = {};
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &transmit;
    t.rx_buffer = &receive;

    // Perform SPI msg transaction
    ret = spi_device_transmit(_spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU successfully transmitted initial message!");
    }
    else {
        ESP_LOGE(TAG, "IMU initial transmission failed!");
        return ret;
    }

    // Check if the right message was received
    if (receive == 0x70) {
        ESP_LOGE(TAG, "LSM6DSV16X detected! ID: 0x%02X", receive);
    }
    else {
        ESP_LOGE(TAG, "Device mismatch! Expected 0x70, got 0x%02X", receive);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

