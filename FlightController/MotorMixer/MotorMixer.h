#include "mbed.h"
#include "Global.h"
#include "PidWrapper.h"
#include "NavigationController.h"

#ifndef MotorMixer_H
#define MotorMixer_H

class MotorMixer
{
  public:                 
    MotorMixer(PinName motor1, PinName motor2, PinName motor3, PinName motor4);   
    ~MotorMixer();
    
    struct MotorPower
    {
       double motor1;
       double motor2;
       double motor3;
       double motor4;
    };
    
    void computePower(PidWrapper::PidOutput pidOutput, double throttle);
    void computePower(double throttle);
    void setPower(double motorPower);
    void setPower(double motor1Power, double motor2Power, double motor3Power, double motor4Power);
    void setPower(MotorMixer::MotorPower motorPower);
    MotorPower getMotorPower();
    
  private:
    PwmOut*  _motor1;
    PwmOut*  _motor2;
    PwmOut*  _motor3;
    PwmOut*  _motor4;
    
    MotorPower _motorPower; 
};

#endif