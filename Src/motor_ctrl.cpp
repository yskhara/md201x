/*
 * motor_ctrl.cpp
 *
 *  Created on: Dec 23, 2018
 *      Author: yusaku
 */

#include "motor_ctrl.hpp"

void MotorCtrl::Control(void)
{

    if((GPIOC->IDR & GPIO_IDR_IDR14) == 0)
    {
//#warning "ignore me if you know what you are doing."
        this->Shutdown();
    }

    if(this->shutdown)
    {
        // disable gate drivers
        GPIOB->BSRR = GPIO_BSRR_BR15;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;

        // turn red led on, yellow off
        GPIOB->BSRR = GPIO_BSRR_BS2 | GPIO_BSRR_BR1;

        this->ResetState();

        return;
    }

    // flash yellow led, red off
    GPIOB->BSRR = GPIO_BSRR_BR2 | GPIO_BSRR_BS1;


    this->error_prev = this->error;
    this->pulse = static_cast<int16_t>(TIM2->CNT);
    TIM2->CNT = 0;
    //this->pulse = this->enc_cnt;
    //this->enc_cnt = 0;
    this->velocity = pulse * Kh;
    this->error = this->target_velocity - this->velocity;

    this->u_p = Kp * (this->error - this->error_prev);
    this->u_i = KiTc * this->error;

    this->target_torque += (u_p + u_i);

    // limit torque
    if (MaximumTorque < target_torque)
    {
        target_torque = MaximumTorque;
    }
    else if (target_torque < -MaximumTorque)
    {
        target_torque = -MaximumTorque;
    }

    target_voltage = (target_torque * Kg) + (this->velocity * Ke);

    if (MaximumVoltage < target_voltage)
    {
        target_voltage = MaximumVoltage;
    }
    else if (target_voltage < -MaximumVoltage)
    {
        target_voltage = -MaximumVoltage;
    }

    // apply output voltage
    SetDuty(target_voltage * 1000 / MaximumVoltage);

    GPIOB->BSRR = GPIO_BSRR_BR1;
}

void MotorCtrl::SetTarget(double target)
{
    double tmp = target * Kr;

    if (MaximumVelocity < tmp)
    {
        this->target_velocity = MaximumVelocity;
    }
    else if (tmp < -MaximumVelocity)
    {
        this->target_velocity = -MaximumVelocity;
    }
    else
    {
        this->target_velocity = tmp;
    }
}

void MotorCtrl::Print(void)
{
    char buf[128];
    int ret = sprintf(buf, "%06lu,%+03d,%+3.3lf,%+3.3lf,%+3.3lf,%+3.3lf,%+3.3lf,%+3.3lf,%+3.3lf,%+3.3lf\r\n", HAL_GetTick(),
            this->pulse, this->velocity, this->target_velocity, this->error, this->error_prev, this->u_p, this->u_i,
            this->target_torque, this->target_voltage);

    if (ret < 0)
    {
        return;
    }

    serial.write((const uint8_t *) buf, ret);
}


void MotorCtrl::ReadConfig(void)
{
    this->Kp = confStruct.Kp;
    this->KiTc = confStruct.KiTc;
    this->Ke = confStruct.Ke;
    this->Kg = confStruct.Kg;
    this->Kh = confStruct.Kh;
    this->Kr = confStruct.Kr;
    this->MaximumVelocity = confStruct.MaxVel;
    this->MaximumTorque = confStruct.MaxTrq;
    this->SetSupplyVoltage(confStruct.Vsup);
}

void MotorCtrl::WriteConfig(void)
{
    confStruct.Kp = this->Kp;
    confStruct.KiTc = this->KiTc;
    confStruct.Ke = this->Ke;
    confStruct.Kg = this->Kg;
    confStruct.Kh = this->Kh;
    confStruct.Kr = this->Kr;
    confStruct.MaxVel = this->MaximumVelocity;
    confStruct.MaxTrq = this->MaximumTorque;
    confStruct.Vsup = this->SupplyVoltage;
}

