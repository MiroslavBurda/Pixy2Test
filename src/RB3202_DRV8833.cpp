#include "RB3202_DRV8833.hpp"

namespace rb3202
{
//public:
void DRV8833::setDriver()
{
    driver_pins_t _driverPins;
    _driverPins.pwm_pin[0] = MOTOR_PWM0_GPIO;
    _driverPins.pwm_pin[1] = MOTOR_PWM1_GPIO;
    _driverPins.pwm_pin[2] = MOTOR_PWM2_GPIO;
    _driverPins.pwm_pin[3] = MOTOR_PWM3_GPIO;
    _driverPins.sleep_pin = MOTOR_SLEEP_GPIO;
    setPins(_driverPins);
}

void DRV8833::setChannelPower(bool channel, float power)
{
    internalSetChannelPower(channel, (MAX_PWM - int((power*(-0.01))*MAX_PWM)));
}

//protected:
void DRV8833::setPins(driver_pins_t pins)
{
    setMotorSleepPin(pins.sleep_pin);
    setMotorPwmPins(pins.pwm_pin[0], 0);
    setMotorPwmPins(pins.pwm_pin[1], 1);
    setMotorPwmPins(pins.pwm_pin[2], 2);
    setMotorPwmPins(pins.pwm_pin[3], 3);
}

void DRV8833::setMotorPwmPins(gpio_num_t pin, uint8_t channel)
{
    ledc_timer_config_t pwm_timer;

    pwm_timer.duty_resolution = PWM_RESOLUTION;
    pwm_timer.freq_hz = FREGUENCY;
    pwm_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_timer.timer_num = ledc_timer_t(channel);

    ledc_channel_config_t pwm_channel;
    pwm_channel.channel = ledc_channel_t(channel);
    pwm_channel.duty = 0;
    pwm_channel.gpio_num = pin;
    pwm_channel.hpoint = 0;
    pwm_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_channel.timer_sel = ledc_timer_t(channel);

    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(channel), 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(channel));
}

void DRV8833::setMotorSleepPin(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, true);
}

void DRV8833::internalSetChannelPower(bool channel, int power)
{
    if(!channel)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(0), setPwmPercent(power,0));
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(1), setPwmPercent(power,1));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(0));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(1));
    }
    else
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(2), setPwmPercent(power,2));
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(3), setPwmPercent(power,3));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(2));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_t(3));
    }
}

//private:

int DRV8833::setPwmPercent(float percent, int channel)
{
    if(percent < 0)
    {
        if((channel==1)||(channel==3))
            return MAX_PWM;
        else
            return MAX_PWM - percent;
    }
    else
    {
        if((channel==1)||(channel==3))
            return MAX_PWM - percent;
        else
            return MAX_PWM;
    }
}


}
