#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"

// 定义 PWM 通道和引脚
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_FREQ_HZ 5000 // PWM 输出频率
#define LEDC_CHANNEL_RED LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE LEDC_CHANNEL_2
#define LEDC_GPIO_RED 9
#define LEDC_GPIO_GREEN 10
#define LEDC_GPIO_BLUE 11

// 初始化 PWM
void ledc_init(void)
{
    // 配置 PWM 定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit resolution
        .freq_hz = LEDC_OUTPUT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // 配置红色通道
    ledc_channel_config_t ledc_channel_red = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RED,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_RED,
        .duty = 0, // 初始占空比
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_red);

    // 配置绿色通道
    ledc_channel_config_t ledc_channel_green = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_GREEN,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_GREEN,
        .duty = 0, // 初始占空比
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_green);

    // 配置蓝色通道
    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_BLUE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_BLUE,
        .duty = 0, // 初始占空比
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_blue);
}

// 设置通道占空比
void set_channel_duty(ledc_channel_t channel, uint8_t duty)
{
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

// 更新颜色通道
void update_color_channel(uint8_t* value, bool* increment, uint8_t step)
{
    if (*increment) {
        *value += step;
        if (*value >= 255) {
            *value = 255;
            *increment = false;
        }
    } else {
        *value -= step;
        if (*value <= 0) {
            *value = 0;
            *increment = true;
        }
    }
}