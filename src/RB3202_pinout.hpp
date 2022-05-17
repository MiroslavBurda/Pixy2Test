#pragma once

#include <stdio.h>

#include <driver/adc.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define RB3202_BOARD        0

#define LOW                 0
#define HIGH                1
#define PI                  3.1415926535897932384

namespace rb3202
{
    static const gpio_num_t BOARD_OFF_GPIO = GPIO_NUM_13;
    static const gpio_num_t BOARD_BATREF_GPIO = GPIO_NUM_35;

    static const gpio_num_t MOTOR_PWM0_GPIO = GPIO_NUM_0;
    static const gpio_num_t MOTOR_PWM1_GPIO = GPIO_NUM_16;
    static const gpio_num_t MOTOR_PWM2_GPIO = GPIO_NUM_5;
    static const gpio_num_t MOTOR_PWM3_GPIO = GPIO_NUM_17;
    static const gpio_num_t MOTOR_SLEEP_GPIO = GPIO_NUM_2;

    static const gpio_num_t ENC_A0_GPIO = GPIO_NUM_4;
    static const gpio_num_t ENC_B0_GPIO = GPIO_NUM_15;
    static const gpio_num_t ENC_A1_GPIO = GPIO_NUM_18;
    static const gpio_num_t ENC_B1_GPIO = GPIO_NUM_19;

    static const gpio_num_t SW_0_GPIO = GPIO_NUM_25;
    static const gpio_num_t SW_1_GPIO = GPIO_NUM_12;
    static const gpio_num_t SW_2_GPIO = GPIO_NUM_27;
    static const gpio_num_t SW_3_GPIO = GPIO_NUM_26;

    static const gpio_num_t LED_R_GPIO = GPIO_NUM_21;  // in this order; Red is the nearest of the border
    static const gpio_num_t LED_G_GPIO = GPIO_NUM_22;
    static const gpio_num_t LED_B_GPIO = GPIO_NUM_23;

    static const gpio_num_t SERVO_SMART_GPIO = GPIO_NUM_35; // on the board is wrong description; GPIO is on the connector for smartservo pin the nearest of the border

    struct driver_pins_t
    {
        gpio_num_t pwm_pin[4] = {
            MOTOR_PWM0_GPIO,
            MOTOR_PWM1_GPIO,
            MOTOR_PWM2_GPIO,
            MOTOR_PWM3_GPIO,
        };

        gpio_num_t sleep_pin = rb3202::MOTOR_SLEEP_GPIO;
    };
};