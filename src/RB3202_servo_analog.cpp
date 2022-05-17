#include <esp_log.h>

#include "RB3202_servo_analog.hpp"

namespace rb3202
{

ServoAnalog::ServoAnalog(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel, float initialAngle) : m_channel(channel) {
    ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .bit_num = LEDC_TIMER_15_BIT,
        .timer_num = timer,
        .freq_hz = 50,
        .clk_cfg = {},
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    ledc_channel_config_t config = {
        .gpio_num = pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = getDutyForAngle(initialAngle),
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

ServoAnalog::~ServoAnalog() {
    ledc_stop(LEDC_HIGH_SPEED_MODE, m_channel, 0);
}

uint32_t ServoAnalog::getDutyForAngle(float angle) const {
    if(angle < 0.f || angle > 180.f) {
        ESP_LOGE("ServoAnalog", "invalid servo angle: %f, ignoring", angle);
        return MIN_ANGLE_US;
    }

    uint32_t time = MIN_ANGLE_US + (MAX_ANGLE_US - MIN_ANGLE_US)*(angle / 180.f);
    return getDutyForTime(time);
}

void ServoAnalog::set(float angle) {
    auto duty = getDutyForAngle(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channel));
}

};
