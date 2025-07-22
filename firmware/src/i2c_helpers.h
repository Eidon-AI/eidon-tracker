#pragma once
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000

static inline void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static inline esp_err_t i2c_write(uint8_t dev_addr, uint8_t *data, size_t len) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, data, len, 1000 / portTICK_PERIOD_MS);
}

static inline esp_err_t i2c_read(uint8_t dev_addr, uint8_t *data, size_t len) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, dev_addr, data, len, 1000 / portTICK_PERIOD_MS);
} 