/**
 * @file led.c
 * @brief RGB LED status indicator module implementation
 * @version 1.0
 * @date 2026-01-29
 */

#include "led.h"
#include "tim.h"
#include "string.h"

static LEDInstance led_instance = {0};

/**
 * @brief 初始化LED模块
 */
void LEDInit(void)
{
    // 注册红色LED (TIM5_CH3, PH12)
    PWM_Init_Config_s led_r_config = {
        .htim = &htim5,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001,  // 1kHz PWM频率
        .callback = NULL,
        .id = NULL
    };
    led_instance.led_r = PWMRegister(&led_r_config);

    // 注册绿色LED (TIM5_CH2, PH11)
    PWM_Init_Config_s led_g_config = {
        .htim = &htim5,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0,
        .period = 0.001,
        .callback = NULL,
        .id = NULL
    };
    led_instance.led_g = PWMRegister(&led_g_config);

    // 注册蓝色LED (TIM5_CH1, PH10)
    PWM_Init_Config_s led_b_config = {
        .htim = &htim5,
        .channel = TIM_CHANNEL_1,
        .dutyratio = 0,
        .period = 0.001,
        .callback = NULL,
        .id = NULL
    };
    led_instance.led_b = PWMRegister(&led_b_config);

    led_instance.status = LED_STATUS_OFF;
    led_instance.blink_count = 0;
    led_instance.brightness = 0.3f;  // 默认亮度30%（避免太亮）
}

/**
 * @brief 设置LED状态
 */
void LEDSetStatus(LED_Status_e status)
{
    led_instance.status = status;
    led_instance.blink_count = 0;
}

/**
 * @brief 设置LED亮度
 */
void LEDSetBrightness(float brightness)
{
    if (brightness < 0.0f)
        brightness = 0.0f;
    if (brightness > 1.0f)
        brightness = 1.0f;
    led_instance.brightness = brightness;
}

/**
 * @brief 设置RGB LED颜色
 * @param r 红色亮度 (0.0~1.0)
 * @param g 绿色亮度 (0.0~1.0)
 * @param b 蓝色亮度 (0.0~1.0)
 */
static void LEDSetRGB(float r, float g, float b)
{
    PWMSetDutyRatio(led_instance.led_r, r * led_instance.brightness);
    PWMSetDutyRatio(led_instance.led_g, g * led_instance.brightness);
    PWMSetDutyRatio(led_instance.led_b, b * led_instance.brightness);
}

/**
 * @brief LED任务，需要周期性调用以实现闪烁效果
 */
void LEDTask(void)
{
    if (led_instance.status == LED_STATUS_GREEN_ON) {
        LEDSetRGB(0, 1.0f, 0);
    } else {
        LEDSetRGB(0, 0, 0);
    }
}
