#include "RB3202_stepr_motor.hpp"

namespace rb3202
{
//private:
void stepper_motor::setCoil(int state, int coil)
{
    switch (state)
    {
    case -1:
        gpio_set_level(pins.pwm_pin[0+(2*coil)], true);
        gpio_set_level(pins.pwm_pin[1+(2*coil)], false);
        break;
    case 0:
        gpio_set_level(pins.pwm_pin[0+(2*coil)], false);
        gpio_set_level(pins.pwm_pin[1+(2*coil)], false);
        break;
    case 1:
        gpio_set_level(pins.pwm_pin[0+(2*coil)], false);
        gpio_set_level(pins.pwm_pin[1+(2*coil)], true);
        break;
    default:
        break;
    }
}

void stepper_motor::backStep(int step_time)
{
    const TickType_t x_delay = int(step_time / (4*portTICK_PERIOD_MS));

    setCoil(0, 0);
    setCoil(-1, 1);
    vTaskDelay(x_delay);

    setCoil(-1, 0);
    setCoil(0, 1);
    vTaskDelay(x_delay);

    setCoil(0, 0);
    setCoil(1, 1);
    vTaskDelay(x_delay);

    setCoil(1, 0);
    setCoil(0, 1);
    vTaskDelay(x_delay); 
}

void stepper_motor::frontStep(int step_time)
{
    const TickType_t x_delay = int(step_time / (4*portTICK_PERIOD_MS));

    setCoil(1, 0);
    setCoil(0, 1);
    vTaskDelay(x_delay);

    setCoil(0, 0);
    setCoil(1, 1);
    vTaskDelay(x_delay);

    setCoil(-1, 0);
    setCoil(0, 1);
    vTaskDelay(x_delay);

    setCoil(0, 0);
    setCoil(-1, 1);
    vTaskDelay(x_delay); 
}

void stepper_motor::setMotorPin(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, true);
}

void stepper_motor::stepControl(void *self_)
{
    auto *self = (stepper_motor*)self_;
    int x = 0;

    while(true)
    {
        if(self->speet > 0)
        {
            self->frontStep(10000/self->speet);
        }
        else if(self->speet < 0)
        {
            self->backStep(-10000/self->speet);
        }
        x++;
        printf("numbr of stepp: %d \n", x);
    }
}

//protected:


//public:
void stepper_motor::setAllPins()
{
    pins.pwm_pin[0] = MOTOR_PWM0_GPIO;
    pins.pwm_pin[1] = MOTOR_PWM1_GPIO;
    pins.pwm_pin[2] = MOTOR_PWM2_GPIO;
    pins.pwm_pin[3] = MOTOR_PWM3_GPIO;
    pins.sleep_pin = MOTOR_SLEEP_GPIO;

    setMotorPin(pins.pwm_pin[0]);
    setMotorPin(pins.pwm_pin[1]);
    setMotorPin(pins.pwm_pin[2]);
    setMotorPin(pins.pwm_pin[3]);
    setMotorPin(pins.sleep_pin);
}

void stepper_motor::doStep(int number_of_steps, int step_time)
{
    if(step_time > 0)
    {
        for(int a = 0; a < number_of_steps; a++)
        {
            frontStep(step_time);
        }
    }
    else if(step_time < 0)
    {
        step_time = 0-step_time;
        for(int a = 0; a < number_of_steps; a++)
        {
            backStep(step_time);
        }
    }
}

bool stepper_motor::set()
{
    setAllPins();
    TaskHandle_t xHandle = NULL;
    xTaskCreate(stepControl, "stepper", 4096, this, 2, &xHandle);
    return true;
}

}