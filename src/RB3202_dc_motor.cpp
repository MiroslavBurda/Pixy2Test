#include <stdio.h>

#include "RB3202_dc_motor.hpp"
#include "RB3202_pinout.hpp"
#include "RB3202_DRV8833.hpp"
#include "RB3202_encoder.hpp"

namespace rb3202
{
//private:
void DC_motor::PIDProces(void *self_)
{
    auto *self = (DC_motor*)self_;

    const TickType_t x_delay = COUNT_PID_PERIOD / portTICK_PERIOD_MS;
    RB3202_encoder encoder (self->pid_data.enc_pin[0], self->pid_data.enc_pin[1]);
    encoder.init();
    while (true)
    {
        self->pid_data.enc_position = encoder.getCount();
        self->pid_data.virtual_wheel += ((self->pid_data.rotate*COUNT_PID_PERIOD)/1000)*NUMBER_OF_PULS_PER_REVOLUTION;
        self->setWheelPower();
        vTaskDelay(x_delay);
    }
}

void DC_motor::choicePins()
{
    if(pid_data.motor == 0)
    {
        pid_data.enc_pin[0] = rb3202::ENC_A0_GPIO;
        pid_data.enc_pin[1] = rb3202::ENC_B0_GPIO;
    }
    else if(pid_data.motor == 1)
    {
        pid_data.enc_pin[0] = rb3202::ENC_A1_GPIO;
        pid_data.enc_pin[1] = rb3202::ENC_B1_GPIO;
    }
}

void DC_motor::setWheelPower()
{
    pid_data.en = pid_data.enc_position - pid_data.virtual_wheel;
    pid_data.power = cropExtremeValues(pid_data.power+count_P()+count_D());
    internalSetChannelPower(pid_data.motor, int(pid_data.power));
    printf("enc: %d power: %4.1f virtual_wheel: %4.4f en: %4.4f P: %4.4f I: %4.4f D: %4.4f", 
            pid_data.enc_position, pid_data.power, pid_data.virtual_wheel, 
            pid_data.en, count_P(), count_I(), count_D());
    printf(" c-P %4.4f cI: %4.4f cD: %4.4f \n",pid_const.P, pid_const.I, pid_const.D);
}

float DC_motor::count_P()
{
    return pid_data.en * 0.001f;
}

float DC_motor::count_I()
{
    float I_membr = 0;
    for(int a = 1000/COUNT_PID_PERIOD - 1; a > 0;a--)
    {
        pid_data.I_memori[a] += pid_data.I_memori[a-1];
    }
    pid_data.I_memori[0] = pid_data.en;
    for(int a = 0; a < 1000/COUNT_PID_PERIOD;a++)
    {
        I_membr += pid_data.I_memori[a];
    }
    return I_membr * pid_const.I;
}

float DC_motor::count_D()
{
    return (pid_data.en - pid_data.I_memori[1])*pid_const.D;
}

float DC_motor::cropExtremeValues(float x, int extrem)
{
    if((x<extrem)&&(x>(extrem*(-1))))
        return x;
    else if(x<0)
    {
        return (extrem*(-1));
    }
    else
    {
        return extrem;
    }
}

//protected:
void DC_motor::adjustConstants(pid_constant_t data)
{
    pid_const.P = data.P;
    pid_const.I = data.I;
    pid_const.D = data.D;
}

pid_data_t DC_motor::getPidData()
{
    return pid_data;
}

//public:
DC_motor::DC_motor()
{
    //pid_const.P = 0.01;
    pid_const.I = 0;
    pid_const.D = 0.00001;
}

void DC_motor::sedRotate(float rotate)
{
    pid_data.rotate = rotate;
}

void DC_motor::sedPID(int motor)
{
    pid_data.motor = motor;
    choicePins();
    setDriver();
    TaskHandle_t xHandle = NULL;

    xTaskCreate(PIDProces , "PID", 4096, this, 2, &xHandle);
}

} // namespace rb3202
