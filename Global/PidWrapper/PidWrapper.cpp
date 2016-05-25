#include "PidWrapper.h"

PidWrapper::PidWrapper(){}

PidWrapper::~PidWrapper(){}

bool PidWrapper::initialise(PidParameter pidParameter, double inputMin, double inputMax, double outputMin, double outputMax, float updateTime)
{
    _pid = new PID(pidParameter.p, pidParameter.i, pidParameter.d, updateTime);
    _pid->setInputLimits(inputMin, inputMax);
    _pid->setOutputLimits(outputMin, outputMax);
    _pid->setMode(AUTO_MODE);
    _pid->setSetPoint(0.0);
    _pid->setBias(0);
    
    DEBUG("PID wrapper initialised\r\n");
    return true; 
}

double PidWrapper::compute(double setPoint, double processValue)
{
    _pid->setSetPoint(setPoint);
    _pid->setProcessValue(processValue);
    return _pid->compute();   
}

PidWrapper::PidParameter  PidWrapper::getPidParameters()
{
    PidWrapper::PidParameter pidParameters;
    pidParameters.p = _pid->getPParam();    
    pidParameters.i = _pid->getIParam(); 
    pidParameters.d = _pid->getDParam(); 
    
    return pidParameters;
}

void PidWrapper::reset()
{
    _pid->reset();
}

void PidWrapper::setPidParameters(PidWrapper::PidParameter pidParameters)
{
    _pid->setTunings(pidParameters.p, pidParameters.i, pidParameters.d);
    DEBUG("P %1.8f, I %1.8f, D %1.8f\r\n", pidParameters.p, pidParameters.i, pidParameters.d);
}

void PidWrapper::setBias(float bias)
{
    _pid->setBias(bias);
}