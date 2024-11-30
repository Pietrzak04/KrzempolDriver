#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio_filter.h"

#define APP_TAG "driver"

//GPIO config
#define TRIAC_OUTPUT 7
#define ZERO_ZROSS 40


static void IRAM_ATTR zero_cross_int(void* arg)
{
    static uint8_t status;

    status ^= 1;
    gpio_set_level(TRIAC_OUTPUT, status);
}

void config_gpio(int *counter)
{
    gpio_install_isr_service(0);

    gpio_config_t output_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TRIAC_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&output_conf);

    gpio_config_t input_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL<<ZERO_ZROSS),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&input_conf);

    gpio_isr_handler_add(ZERO_ZROSS, zero_cross_int, (void*) counter);
}

void app_main(void)
{
    int int_counter = 0;

    config_gpio(&int_counter);
    gpio_dump_io_configuration(stdout, 1ULL << ZERO_ZROSS);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    while(true)
    {
        ESP_LOGI(APP_TAG, "counter %d, %lld", int_counter, esp_timer_get_time());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}