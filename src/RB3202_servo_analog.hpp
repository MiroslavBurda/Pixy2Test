#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

namespace rb3202
{

class ServoAnalog {
public:
    ServoAnalog(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel, float initialAngle = 90.f);

    ServoAnalog(const ServoAnalog&) = delete;

    ~ServoAnalog();

    void set(float angle);

private:
    static constexpr uint32_t MIN_ANGLE_US = 1000;
    static constexpr uint32_t MAX_ANGLE_US = 2000;

    uint32_t getDutyForAngle(float angle) const;
    uint32_t getDutyForTime(uint32_t timeUs) const { return (1<<15) * timeUs / 20000; }

    ledc_channel_t m_channel;
};

};
