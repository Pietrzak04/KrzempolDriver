#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"

#define APP_TAG "driver"

//GPIO config
#define TRIAC_OUTPUT 7
#define ZERO_ZROSS 40

uint64_t time_value = 9000;

gptimer_handle_t triac_timer = NULL;
TaskHandle_t triac_task_handle = NULL;

static void IRAM_ATTR zero_cross_int(void* arg)
{
    static uint64_t time;

    if (esp_timer_get_time() - time > 500)
    {
        ESP_ERROR_CHECK(gptimer_set_raw_count(triac_timer, 0));
        ESP_ERROR_CHECK(gptimer_start(triac_timer));
    }

    time = esp_timer_get_time();
}

static bool IRAM_ATTR triac_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    uint64_t time = 0;
    gptimer_stop(timer);

    gpio_set_level(TRIAC_OUTPUT, true);
    
    time = esp_timer_get_time();
    while (esp_timer_get_time() - time < 30);

    gpio_set_level(TRIAC_OUTPUT, false);

    return (high_task_awoken == pdTRUE);
}

void config_gpio()
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
        .pull_up_en = 1
    };
    gpio_config(&input_conf);

    gpio_isr_handler_add(ZERO_ZROSS, zero_cross_int, NULL);
}

void config_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &triac_timer));

    gptimer_event_callbacks_t alarm = {
        .on_alarm = triac_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(triac_timer, &alarm, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = time_value,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(triac_timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(triac_timer));
}

void app_main(void)
{
    config_timer();
    config_gpio();
}