#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <u8g2.h>
#include <driver/i2c.h>
#include "esp_timer.h"

#include "pinout.h"
#include "u8g2_hal.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ          50000
#define I2C_TIMEOUT_MS              1000
#define I2C_ACK_CHECK_EN            0x1
#define I2C_ACK_CHECK_DIS           0x0

static u8g2_t display;

static uint32_t freq_list[] = {
    10000,
    50000,
    100000,
    400000,
    1000000,
    1500000,
};

typedef enum {
    MODE_CONTROLLER,
    MODE_ENDPOINT,
} example_mode_t;

typedef struct {
    example_mode_t mode;
    uint8_t scan_addr;
    uint8_t write_buf[4];
    uint8_t read_buf[4];
    uint32_t i2c_freq_hz;
    bool read_ok;
    bool write_ok;
    uint8_t freq_list_idx;
} example_state_t;

uint8_t i2c_scan_bus(void);
bool i2c_example_master_read(uint8_t address, uint8_t* buf, size_t buf_len);
bool i2c_example_master_write(uint8_t address, uint8_t* buf, size_t buf_len);
void update_display(example_state_t state);

/**
 * This example creates either an I2C Echo endpoint or a
 * scanner which searches for the first available I2C device,
 * then expects an echo device to test round-trip integrity.
 */
void app_main(void) {
    example_state_t state = {
        .mode = MODE_CONTROLLER,
        .scan_addr = 0x00,
        .write_buf = { 0x00, 0x00, 0x00, 0x00 },
        .read_buf = { 0x00, 0x00, 0x00, 0x00 },
        .read_ok = false,
        .write_ok = false,
        .freq_list_idx = 0,
        .i2c_freq_hz = freq_list[0],
    };
    
    // Check Mode Switch
    gpio_set_direction(MODE_SELECT_GPIO, GPIO_MODE_INPUT);

    if (1 == gpio_get_level(MODE_SELECT_GPIO)) {
        state.mode = MODE_ENDPOINT;
        state.scan_addr = 0x39 << 1;
    } else {
        state.mode = MODE_CONTROLLER;
    }

    // Setup Display
    u8g2_hal_config_t config = {
        .sda_gpio = DISPLAY_SDA_GPIO,
        .scl_gpio = DISPLAY_SCL_GPIO,

        // One of my boards is broken and missing half the page memory??
        .use_alt_driver = state.mode == MODE_ENDPOINT, 
    };

    u8g2_hal_init(&display, &config);

    u8g2_SetFont(&display, u8g2_font_8x13_mf);
    u8g2_SetDrawColor(&display, 1);

    // Initialize I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_EXAMPLE_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_EXAMPLE_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    size_t i2c_rx_buf_len = I2C_MASTER_RX_BUF_DISABLE;
    size_t i2c_tx_buf_len = I2C_MASTER_TX_BUF_DISABLE;

    if (state.mode == MODE_CONTROLLER) {
        i2c_config.master.clk_speed = state.i2c_freq_hz;
    } else {
        i2c_config.mode = I2C_MODE_SLAVE;
        i2c_config.slave.slave_addr = state.scan_addr;
        i2c_config.slave.addr_10bit_en = 0;

        i2c_rx_buf_len = 256;
        i2c_tx_buf_len = 256;
    }


    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, i2c_rx_buf_len, i2c_tx_buf_len, 0));

    update_display(state);
    TickType_t delay_ms = 1000;

    // Main Loop
    for (;;) {
        if (state.scan_addr == 0x00) {
            state.scan_addr = i2c_scan_bus();
        } else {
            if (state.mode == MODE_CONTROLLER) {
                if (!state.write_ok) {
                    for (size_t i = 0; i < 4; i++) state.write_buf[i] = rand();
                    state.write_ok = i2c_example_master_write(state.scan_addr, state.write_buf, 4);
                    delay_ms = 500;
                } else if (!state.read_ok) {
                    state.read_ok = i2c_example_master_read(state.scan_addr, state.read_buf, 4);
                    delay_ms = 2000;
                } else {
                    state.write_ok = false;
                    state.read_ok = false;

                    // Step through frequency:
                    state.freq_list_idx = (state.freq_list_idx + 1) % (sizeof(freq_list) / sizeof(freq_list[0]));
                    state.i2c_freq_hz = freq_list[state.freq_list_idx];
                    i2c_config.master.clk_speed = state.i2c_freq_hz;
                    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
                    state.scan_addr = i2c_scan_bus();
                    delay_ms = 100;
                }
            } else {
                // Endpoint code not yet set.
                size_t read = i2c_slave_read_buffer(I2C_MASTER_NUM, state.read_buf, 4, 100);
                delay_ms = 100;

                if (read > 0) {
                    state.read_ok = true;

                    printf("Read %d bytes: 0x", read);
                    for (size_t i = 0; i < read; i++) {
                        state.write_buf[i] = state.read_buf[i];
                        printf("%02x", state.read_buf[i]);
                    }
                    printf("\n");

                    i2c_reset_tx_fifo(I2C_MASTER_NUM);
                    state.write_ok = i2c_slave_write_buffer(I2C_MASTER_NUM, state.write_buf, read, 100) == read;
                }
            }
        }

        update_display(state);
        vTaskDelay(delay_ms / portTICK_RATE_MS);
    }
}

void update_display(example_state_t state) {
    char buf[41] = "";
    u8g2_ClearBuffer(&display);

    if (state.mode == MODE_CONTROLLER) {
        snprintf(buf, 40, "Host   @ %4dKHz", state.i2c_freq_hz / 1000);
        u8g2_DrawStr(&display, 0, 13 * 1, buf);

        if (state.scan_addr == 0x00) {
            snprintf(buf, 40, "SCAN: - wait -");
            u8g2_DrawStr(&display, 0, 14 * 2, buf);
        } else {
            snprintf(buf, 40, "SCAN: 0x%02x", state.scan_addr);
            u8g2_DrawStr(&display, 0, 14 * 2, buf);

            if (state.write_ok) {
                snprintf(buf, 40, "WRTE: 0x%02x%02x%02x%02x", state.write_buf[0], state.write_buf[1], state.write_buf[2], state.write_buf[3]);
                u8g2_DrawStr(&display, 0, 14 * 3, buf);
            } else {
                snprintf(buf, 40, "WRTE: - wait -");
                u8g2_DrawStr(&display, 0, 14 * 3, buf);
            }

            if (state.read_ok) {
                snprintf(buf, 40, "READ: 0x%02x%02x%02x%02x", state.read_buf[0], state.read_buf[1], state.read_buf[2], state.read_buf[3]);
                u8g2_DrawStr(&display, 0, 14 * 4, buf);
            } else {
                snprintf(buf, 40, "READ: - wait -");
                u8g2_DrawStr(&display, 0, 14 * 4, buf);
            }
        }
    } else {
        snprintf(buf, 40, "Endp Addr:  0x%02x", state.scan_addr);
        u8g2_DrawStr(&display, 0, 13 * 1, buf);

        if (state.read_ok) {
            snprintf(buf, 40, "READ: 0x%02x%02x%02x%02x", state.read_buf[0], state.read_buf[1], state.read_buf[2], state.read_buf[3]);
            u8g2_DrawStr(&display, 0, 14 * 2, buf);
        } else {
            snprintf(buf, 40, "READ: - wait -");
            u8g2_DrawStr(&display, 0, 14 * 2, buf);
        }
    }

    u8g2_SendBuffer(&display);
}

uint8_t i2c_scan_bus(void) {
    uint8_t found_addr = 0;

    for (int i = 3; i < 0x78 && found_addr == 0; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (ESP_OK == err) {
            found_addr = i;
        }

        i2c_cmd_link_delete(cmd);
    }

    return found_addr;
}

bool i2c_example_master_read(uint8_t address, uint8_t* buf, size_t buf_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    if (buf_len > 1) {
        i2c_master_read(cmd, buf, buf_len - 1, I2C_ACK_CHECK_DIS);
    }
    i2c_master_read_byte(cmd, buf + buf_len - 1, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf("Error Reading: %s\n", esp_err_to_name(err));

    return err == ESP_OK;
}

bool i2c_example_master_write(uint8_t address, uint8_t* buf, size_t buf_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write(cmd, buf, buf_len, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf("Error Writing: %s\n", esp_err_to_name(err));

    return err == ESP_OK;
}
