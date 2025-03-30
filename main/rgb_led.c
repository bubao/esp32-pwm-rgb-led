#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

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

// 定义按钮引脚
#define BUTTON_GPIO 0

// 定义颜色结构体
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Color_t;

// 信号量句柄
SemaphoreHandle_t xButtonSemaphore;

// 定时器句柄
TimerHandle_t xLongPressTimer;

// 任务句柄
TaskHandle_t xRedTaskHandle = NULL;
TaskHandle_t xGreenTaskHandle = NULL;
TaskHandle_t xBlueTaskHandle = NULL;

// 按钮状态
volatile bool button_pressed = false;
volatile TickType_t button_press_time = 0;
volatile bool long_press_detected = false;

// 日志标签
static const char* TAG = "RGB_LED";

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
    ledc_channel_config_t ledc_channel_red = { .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RED,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_RED,
        .duty = 0, // 初始占空比
        .hpoint = 0 };
    ledc_channel_config(&ledc_channel_red);

    // 配置绿色通道
    ledc_channel_config_t ledc_channel_green = { .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_GREEN,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_GREEN,
        .duty = 0, // 初始占空比
        .hpoint = 0 };
    ledc_channel_config(&ledc_channel_green);

    // 配置蓝色通道
    ledc_channel_config_t ledc_channel_blue = { .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_BLUE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_GPIO_BLUE,
        .duty = 0, // 初始占空比
        .hpoint = 0 };
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

// 中断服务例程（ISR）
static void IRAM_ATTR button_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 获取当前按钮状态
    int button_state = gpio_get_level(BUTTON_GPIO);
    if (button_state == 0) { // 按钮按下（下降沿）
        button_press_time = xTaskGetTickCountFromISR();
        button_pressed = true;
        // 启动长按定时器
        xTimerStartFromISR(xLongPressTimer, &xHigherPriorityTaskWoken);
    } else if (button_state == 1) { // 按钮释放（上升沿）
        button_pressed = false;
        // 停止长按定时器
        xTimerStopFromISR(xLongPressTimer, &xHigherPriorityTaskWoken);
        // 发送信号量
        xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// 长按定时器回调函数
void long_press_timer_callback(TimerHandle_t xTimer)
{
    if (button_pressed) {
        long_press_detected = true;
        button_pressed = false;
        ESP_LOGI(TAG, "Long press detected");
        // 发送信号量
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

// 控制 RGB LED 任务
void rgb_control_task(void* pvParameters)
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    bool red_increment = true;
    bool green_increment = true;
    bool blue_increment = true;
    bool tasks_paused = false;
    bool long_press_active = false;
    TickType_t last_button_press_time = 0;

    while (1) {
        // 等待信号量
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdTRUE) {
            TickType_t current_time = xTaskGetTickCount();
            TickType_t time_since_last_press = current_time - last_button_press_time;

            if (long_press_detected) {
                // 长按检测到，暂停其他任务并进入闪烁模式
                long_press_detected = false;
                long_press_active = true;
                vTaskSuspend(xRedTaskHandle);
                vTaskSuspend(xGreenTaskHandle);
                vTaskSuspend(xBlueTaskHandle);
                ESP_LOGI(TAG, "Tasks suspended for long press");

                while (1) {
                    set_channel_duty(LEDC_CHANNEL_RED, 255);
                    set_channel_duty(LEDC_CHANNEL_GREEN, 0);
                    set_channel_duty(LEDC_CHANNEL_BLUE, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    set_channel_duty(LEDC_CHANNEL_RED, 0);
                    set_channel_duty(LEDC_CHANNEL_GREEN, 255);
                    set_channel_duty(LEDC_CHANNEL_BLUE, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    set_channel_duty(LEDC_CHANNEL_RED, 0);
                    set_channel_duty(LEDC_CHANNEL_GREEN, 0);
                    set_channel_duty(LEDC_CHANNEL_BLUE, 255);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    // 检查是否再次按下按钮以退出闪烁模式
                    if (xSemaphoreTake(xButtonSemaphore, 0) == pdTRUE) {
                        ESP_LOGI(TAG, "Exit long press mode");
                        long_press_active = false;
                        break;
                    }
                }

                // 恢复其他任务
                vTaskResume(xRedTaskHandle);
                vTaskResume(xGreenTaskHandle);
                vTaskResume(xBlueTaskHandle);
                ESP_LOGI(TAG, "Tasks resumed after long press");
            } else if (time_since_last_press < pdMS_TO_TICKS(300)) {
                // 单击检测到，暂停或继续任务
                if (tasks_paused) {
                    vTaskResume(xRedTaskHandle);
                    vTaskResume(xGreenTaskHandle);
                    vTaskResume(xBlueTaskHandle);
                    ESP_LOGI(TAG, "Tasks resumed");
                } else {
                    vTaskSuspend(xRedTaskHandle);
                    vTaskSuspend(xGreenTaskHandle);
                    vTaskSuspend(xBlueTaskHandle);
                    ESP_LOGI(TAG, "Tasks paused");
                }
                tasks_paused = !tasks_paused;
            }

            last_button_press_time = current_time;
        }

        if (!long_press_active) {
            // 更新红色通道
            update_color_channel(&red, &red_increment, 5);
            // 更新绿色通道
            update_color_channel(&green, &green_increment, 2);
            // 更新蓝色通道
            update_color_channel(&blue, &blue_increment, 3);
            // 设置通道占空比
            set_channel_duty(LEDC_CHANNEL_RED, red);
            set_channel_duty(LEDC_CHANNEL_GREEN, green);
            set_channel_duty(LEDC_CHANNEL_BLUE, blue);
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 延时50毫秒
    }
}

// 红色任务
void red_task(void* pvParameters)
{
    uint8_t duty = 0;
    bool increment = true;
    while (1) {
        if (increment) {
            duty += 5;
            if (duty >= 255) {
                increment = false;
            }
        } else {
            duty -= 5;
            if (duty <= 0) {
                increment = true;
            }
        }
        set_channel_duty(LEDC_CHANNEL_RED, duty);
        vTaskDelay(pdMS_TO_TICKS(40)); // 延时40毫秒
    }
}

// 绿色任务
void green_task(void* pvParameters)
{
    uint8_t duty = 0;
    bool increment = true;
    while (1) {
        if (increment) {
            duty += 5;
            if (duty >= 255) {
                increment = false;
            }
        } else {
            duty -= 5;
            if (duty <= 0) {
                increment = true;
            }
        }
        set_channel_duty(LEDC_CHANNEL_GREEN, duty);
        vTaskDelay(pdMS_TO_TICKS(20)); // 延时20毫秒
    }
}

// 蓝色任务
void blue_task(void* pvParameters)
{
    uint8_t duty = 0;
    bool increment = true;
    while (1) {
        if (increment) {
            duty += 5;
            if (duty >= 255) {
                increment = false;
            }
        } else {
            duty -= 5;
            if (duty <= 0) {
                increment = true;
            }
        }
        set_channel_duty(LEDC_CHANNEL_BLUE, duty);
        vTaskDelay(pdMS_TO_TICKS(10)); // 延时10毫秒
    }
}

void app_main(void)
{
    // 初始化 PWM
    ledc_init();

    // 创建二值信号量
    xButtonSemaphore = xSemaphoreCreateBinary();
    if (xButtonSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // 创建长按定时器
    xLongPressTimer = xTimerCreate("Long Press Timer", // 定时器名称
        pdMS_TO_TICKS(3000), // 定时器周期（3秒）
        pdFALSE, // 一次性定时器
        NULL, // 传递给回调函数的参数
        long_press_timer_callback // 回调函数
    );
    if (xLongPressTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create long press timer");
        return;
    }

    // 配置按钮引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE, // 上升沿和下降沿触发中断
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);

    // 安装中断服务例程
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    // 创建红色任务
    xTaskCreate(red_task, // 任务函数
        "Red Task", // 任务名称
        2048, // 任务堆栈大小
        NULL, // 任务参数
        1, // 任务优先级
        &xRedTaskHandle // 任务句柄
    );

    // 创建绿色任务
    xTaskCreate(green_task, // 任务函数
        "Green Task", // 任务名称
        2048, // 任务堆栈大小
        NULL, // 任务参数
        1, // 任务优先级
        &xGreenTaskHandle // 任务句柄
    );

    // 创建蓝色任务
    xTaskCreate(blue_task, // 任务函数
        "Blue Task", // 任务名称
        2048, // 任务堆栈大小
        NULL, // 任务参数
        1, // 任务优先级
        &xBlueTaskHandle // 任务句柄
    );

    // 创建 RGB 控制任务
    xTaskCreate(rgb_control_task, // 任务函数
        "RGB Control Task", // 任务名称
        4096, // 任务堆栈大小（增加到 4096 字）
        NULL, // 任务参数
        2, // 任务优先级
        NULL // 任务句柄
    );

    // 检查堆栈使用情况
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待一段时间
    ESP_LOGI(TAG, "Red Task Stack High Water Mark: %u words",
        uxTaskGetStackHighWaterMark(xRedTaskHandle));
    ESP_LOGI(TAG, "Green Task Stack High Water Mark: %u words",
        uxTaskGetStackHighWaterMark(xGreenTaskHandle));
    ESP_LOGI(TAG, "Blue Task Stack High Water Mark: %u words",
        uxTaskGetStackHighWaterMark(xBlueTaskHandle));
    ESP_LOGI(TAG, "RGB Control Task Stack High Water Mark: %u words",
        uxTaskGetStackHighWaterMark(NULL));
}