#include "MotorMixer.h"

MotorMixer::MotorMixer(PinName motor1, PinName motor2, PinName motor3, PinName motor4)
{   
    _motorPower = MotorPower();
    
    _motor1 = new PwmOut(motor1);
    _motor2 = new PwmOut(motor2);
    _motor3 = new PwmOut(motor3);
    _motor4 = new PwmOut(motor4);
    
    //Set period
    double period = 1.0 / FLIGHT_CONTROLLER_FREQUENCY;
    _motor1->period(period);
    _motor2->period(period);
    _motor3->period(period);
    _motor4->period(period);
    
    //Disable
    setPower(MOTORS_OFF);
    
    DEBUG("Motor power initialised\r\n");  
}

MotorMixer::~MotorMixer(){}

void MotorMixer::computePower(PidWrapper::PidOutput pidOutput, double throttle)
{    
    //Calculate base power to apply from throttle - returns 1060 at min, 1860 at max
    double basePower = MOTORS_MIN + (throttle * 800);
    
    MotorPower motorPower = MotorPower();
    
    //Map motor power - each PID returns -100 <-> 100
    motorPower.motor1 = basePower + pidOutput.pitch + pidOutput.roll + pidOutput.yaw;
    motorPower.motor2 = basePower + pidOutput.pitch - pidOutput.roll - pidOutput.yaw;
    motorPower.motor3 = basePower - pidOutput.pitch - pidOutput.roll + pidOutput.yaw;
    motorPower.motor4 = basePower - pidOutput.pitch + pidOutput.roll - pidOutput.yaw;
    
    //Specify intial motor power limits
    double motorFix = 0;
    double motorMin = motorPower.motor1;
    double motorMax = motorPower.motor1;
    
    //Check motor power is within limits - if not add/remove constant to all motors to keep motor ratio the same
    if(motorPower.motor1 < motorMin) motorMin = motorPower.motor1;
    if(motorPower.motor1 > motorMax) motorMax = motorPower.motor1;
    if(motorPower.motor2 < motorMin) motorMin = motorPower.motor2;
    if(motorPower.motor2 > motorMax) motorMax = motorPower.motor2;
    if(motorPower.motor3 < motorMin) motorMin = motorPower.motor3;
    if(motorPower.motor3 > motorMax) motorMax = motorPower.motor3;
    if(motorPower.motor4 < motorMin) motorMin = motorPower.motor4;
    if(motorPower.motor4 > motorMax) motorMax = motorPower.motor4;
        
    //Check if min or max is outside of the limits
    if(motorMin < MOTORS_MIN) motorFix = MOTORS_MIN - motorMin;
    else if(motorMax > MOTORS_MAX) motorFix = MOTORS_MAX - motorMax;
    
    //Add/remove constant
    motorPower.motor1 += motorFix;
    motorPower.motor2 += motorFix;
    motorPower.motor3 += motorFix;
    motorPower.motor4 += motorFix;
    
    //Set motor power
    setPower(motorPower);
}

void MotorMixer::computePower(double throttle)
{    
    //Calculate base power to apply from throttle - returns 1060 at min, 1860 at max
    double basePower = MOTORS_MIN + (throttle * 800);
    
    MotorPower motorPower = MotorPower();
    motorPower.motor1 = basePower;
    motorPower.motor2 = basePower;
    motorPower.motor3 = basePower;
    motorPower.motor4 = basePower;
    
    //Set motor power
    setPower(motorPower);
}

void MotorMixer::setPower(double motor1Power, double motor2Power, double motor3Power, double motor4Power)
{
    _motorPower.motor1 = motor1Power;
    _motorPower.motor2 = motor2Power;
    _motorPower.motor3 = motor3Power;
    _motorPower.motor4 = motor4Power;
    
    #ifdef MOTORS_ENABLED
        _motor1->pulsewidth_us(motor1Power);
        _motor2->pulsewidth_us(motor2Power);
        _motor3->pulsewidth_us(motor3Power);
        _motor4->pulsewidth_us(motor4Power);
    #endif
}

void MotorMixer::setPower(double motorPower)
{
    setPower(motorPower, motorPower, motorPower, motorPower);
}

void MotorMixer::setPower(MotorMixer::MotorPower motorPower)
{
    setPower(motorPower.motor1, motorPower.motor2, motorPower.motor3, motorPower.motor4);
}

MotorMixer::MotorPower MotorMixer::getMotorPower()
{
    return _motorPower;
}