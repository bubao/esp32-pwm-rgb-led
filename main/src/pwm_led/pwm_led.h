#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include "driver/ledc.h"

/**
 * @file pwm_led.h
 * @brief 初始化 PWM
 * @note 该文件包含 PWM 初始化和控制函数
 */
void ledc_init(void);

/** 
 * @brief 设置 PWM 通道的占空比
 * @param channel PWM 通道
 * @param duty 占空比（0-255）
 */
void set_channel_duty(ledc_channel_t channel, uint8_t duty);

/**
 * @brief 更新颜色通道
 * @param value 颜色值
 * @param increment 增量标志
 * @param step 步长
 * @note 该函数用于更新颜色通道的值
 */
void update_color_channel(uint8_t* value, bool* increment, uint8_t step);

#endif
