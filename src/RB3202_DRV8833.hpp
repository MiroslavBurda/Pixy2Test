#pragma once

#include "RB3202_pinout.hpp"

#define FREGUENCY               3500
#define PWM_RESOLUTION          LEDC_TIMER_8_BIT
#define MAX_PWM                 256

namespace rb3202
{
class DRV8833
{
private:
int setPwmPercent(float percent, int channel);


protected:
void setMotorPwmPins(gpio_num_t pin, uint8_t channel);
void setMotorSleepPin(gpio_num_t pin);
void setPins(driver_pins_t pins);


public:
void setDriver();
void internalSetChannelPower(bool channel, int power);
void setChannelPower(bool channel, float power);

};
}