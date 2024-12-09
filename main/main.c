#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"

#define APP_TAG "driver"

//GPIO config
#define TRIAC_OUTPUT 7
#define ZERO_ZROSS 40
#define ENCODER_GPIO_A 35
#define ENCODER_GPIO_B 36

#define TIME_MIN_LIMIT -1 // -1 is automaticly changed to 0
#define TIME_MAX_LIMIT 400 // 401 is automaticly changed to 0

#define TRIAC_ACTIVATION_TIME 20
#define MAX_WAIT_TIME 10000

//hardware handles
gptimer_handle_t triac_timer = NULL;
pcnt_unit_handle_t pcnt_unit = NULL;

uint64_t wait_time = MAX_WAIT_TIME;

static inline void triac_on_off()
{
    uint64_t time = 0;
    gpio_set_level(TRIAC_OUTPUT, true);

    time = esp_timer_get_time();
    while (esp_timer_get_time() - time < TRIAC_ACTIVATION_TIME);

    gpio_set_level(TRIAC_OUTPUT, false);
}

static void IRAM_ATTR zero_cross_int(void* arg)
{
    static uint64_t time;
    if (esp_timer_get_time() - time > 1000)
    {
        if (wait_time > 9800 || wait_time < 200)
        {
            time = esp_timer_get_time();
            return;
        } 

        ESP_ERROR_CHECK(gptimer_set_raw_count(triac_timer, 0));

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = MAX_WAIT_TIME - wait_time,
        };
        gptimer_set_alarm_action(triac_timer, &alarm_config);

        ESP_ERROR_CHECK(gptimer_start(triac_timer));
    }

    time = esp_timer_get_time();
}

static bool IRAM_ATTR triac_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    gptimer_stop(timer);

    triac_on_off();

    return (high_task_awoken == pdTRUE);
}

static void config_gpio()
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
        .pin_bit_mask = 1ULL << ZERO_ZROSS,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&input_conf);

    gpio_isr_handler_add(ZERO_ZROSS, zero_cross_int, NULL);
}

static void config_timer()
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

    ESP_ERROR_CHECK(gptimer_enable(triac_timer));
}

static void config_encoder()
{
    pcnt_unit_config_t unit_config = {
        .high_limit = TIME_MAX_LIMIT,
        .low_limit = TIME_MIN_LIMIT,
        .flags.accum_count = 0,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 5000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    //encoder channel for increment
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    //encoder channel for decremenmt
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void app_main(void)
{
    config_timer();
    config_gpio();
    config_encoder();

    int read = 0;
    int prev_read = 0;
    float power_percentage = 0;
    while(true)
    {
        while(read == prev_read)
        {
            pcnt_unit_get_count(pcnt_unit, &read);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        prev_read = read;

        power_percentage = (read >> 2) / 100.0f;

        wait_time = (asin(sqrt(power_percentage)) * 2 * MAX_WAIT_TIME) / M_PI;

        ESP_LOGI(APP_TAG, "wartosc %.2f", power_percentage);
    }
}