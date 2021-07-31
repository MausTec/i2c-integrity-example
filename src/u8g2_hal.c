#include <stdio.h>
#include <driver/i2c.h>
#include "u8g2_hal.h"
#include "esp_log.h"

static u8g2_hal_config_t _config;
static i2c_cmd_handle_t  _i2c_cmd;

static const char* TAG = "u8g2_hal";

#define I2C_MASTER_NUM              I2C_NUM_1
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ          50000
#define I2C_TIMEOUT_MS              1000
#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0

uint8_t u8g2_hal_init(u8g2_t* u8g2, u8g2_hal_config_t* config) {
    _config = *config;
    
    if (_config.use_alt_driver) {
        u8g2_Setup_ssd1306_i2c_128x64_alt0_f(u8g2, U8G2_R0, u8g2_hal_byte, u8g2_hal_gpio);
    } else {
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8g2_hal_byte, u8g2_hal_gpio);
    }
    

    u8g2_SetI2CAddress(u8g2, 0x3C << 1);
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    return 1;
}

uint8_t u8g2_hal_gpio(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    // printf("u8g2_hal_gpio[msg: %d, arg_int: %d]\n", msg, arg_int);
    switch (msg) {
    case U8X8_MSG_DELAY_MILLI: {
        vTaskDelay(arg_int / portTICK_PERIOD_MS);
        break;
    }
    }

    return 0;
}

uint8_t u8g2_hal_byte(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    // printf("u8g2_hal_byte[msg: %d, arg_int: %d]\n", msg, arg_int);
    switch (msg) {
    case U8X8_MSG_BYTE_SET_DC: {
        break;
    }

    case U8X8_MSG_BYTE_INIT: {
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = _config.sda_gpio,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = _config.scl_gpio,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };


        ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

        break;
    }

    case U8X8_MSG_BYTE_SEND: {
        uint8_t* data = (uint8_t*) arg_ptr;

        while (arg_int > 0) {
            ESP_ERROR_CHECK(i2c_master_write_byte(_i2c_cmd, *data, ACK_CHECK_EN));
            arg_int--;
            data++;
        }

        break;
    }

    case U8X8_MSG_BYTE_START_TRANSFER: {
        uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
        ESP_LOGD(TAG, "Flushing buffer to addr 0x%02x", i2c_address);

        _i2c_cmd = i2c_cmd_link_create();

        ESP_ERROR_CHECK(i2c_master_start(_i2c_cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(_i2c_cmd, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));

        break;
    }

    case U8X8_MSG_BYTE_END_TRANSFER: {
        ESP_ERROR_CHECK(i2c_master_stop(_i2c_cmd));

        // Send the command sequence:
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, _i2c_cmd, I2C_TIMEOUT_MS / portTICK_RATE_MS));

        i2c_cmd_link_delete(_i2c_cmd);

        break;
    }
    }

    return 0;
}

