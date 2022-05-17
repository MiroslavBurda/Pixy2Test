#pragma once

#include "RB3202_pinout.hpp"

namespace rb3202
{
class stepper_motor
{
private:
int step_position = 0;

driver_pins_t pins;

void setCoil(int state, int coil);

void setMotorPin(gpio_num_t pin);

void frontStep(int step_time);
void backStep(int step_time);

static void stepControl(void *self_);

protected:
int speet = 0;

public:
void doingStep(int _speet)
{
    speet = _speet;
}
void setAllPins();
bool set();
void doStep(int number_of_steps, int step_time);
};

}