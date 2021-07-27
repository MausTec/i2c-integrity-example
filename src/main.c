#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <clib/u8g2.h>
#include "pinout.h"

u8g2_t display;

uint8_t u8x8_gpio_and_delay_esp32(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);

void app_main(void) {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&display, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_esp32);
    u8g2_InitDisplay(&display);
    u8g2_SetPowerSave(&display, 0);

    for (;;) {
        vTaskDelay(1);
    }
}

/**
 * U8g2 HAL routine to support ESP32 Devices using esp-idf native calls.
 */
uint8_t u8x8_gpio_and_delay_esp32(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    switch (msg) {
        // Setup Here
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        break;

        // Handle GPIO Reads
    case U8X8_MSG_GPIO_MENU_SELECT:
        u8x8_SetGPIOResult(u8x8, 0);
        break;

    case U8X8_MSG_GPIO_MENU_NEXT:
        u8x8_SetGPIOResult(u8x8, 0);
        break;

    case U8X8_MSG_GPIO_MENU_PREV:
        u8x8_SetGPIOResult(u8x8, 0);
        break;

    case U8X8_MSG_GPIO_MENU_HOME:
        u8x8_SetGPIOResult(u8x8, 0);
        break;
    }

    return 1;
}