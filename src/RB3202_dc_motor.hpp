#pragma once

#include "RB3202_pinout.hpp"
#include "RB3202_DRV8833.hpp"
#include "RB3202_encoder.hpp"


#define COUNT_PID_PERIOD                100  // controlled power update period
#define NUMBER_OF_PULS_PER_REVOLUTION   1000

namespace rb3202
{

struct pid_data_t
{
    gpio_num_t enc_pin[2];
    int motor = 0;
    float rotate = 0;
    int enc_position = 0;
    float power = 0;
    float virtual_wheel = 0;
    float en = 0;
    float I_memori[1000/COUNT_PID_PERIOD];
    
};

struct pid_constant_t
{
    float P = 0.001;
    float I = 0;
    float D = 0.001;
};

class DC_motor: public DRV8833
{
private:
static void PIDProces(void *this_);
static void rotateWirtualWheel(void *this_);

void choicePins();
void setWheelPower();

float count_P();
float count_I();
float count_D();
float cropExtremeValues(float x, int extrem = 256);

protected:
pid_data_t pid_data;
pid_constant_t pid_const;
void adjustConstants(pid_constant_t data);
pid_data_t getPidData();

public:
DC_motor();

void sedPID(int motor);
void sedRotate(float rotate);

};

}
