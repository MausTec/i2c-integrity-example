#ifndef __u8g2_hal
#define __u8g2_hal

#include <u8g2.h>
#include <driver/gpio.h>

struct u8g2_hal_config {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    gpio_num_t dc_gpio;
    bool use_alt_driver;
};

typedef struct u8g2_hal_config u8g2_hal_config_t;

uint8_t u8g2_hal_init(u8g2_t* u8g2, u8g2_hal_config_t *config);

uint8_t u8g2_hal_gpio(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);
uint8_t u8g2_hal_byte(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);

#endif