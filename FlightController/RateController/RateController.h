#include "mbed.h"
#include "Global.h"
#include "PidWrapper.h"
#include "ConfigFileWrapper.h"
#include "NavigationController.h"

#ifndef RateController_H
#define RateController_H

class RateController               
{
  public:                  
    RateController(Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper); 
    ~RateController();
    
    PidWrapper::PidOutput compute();
    void reset();
    void setYawRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setPitchRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setRollRatePidParameters(PidWrapper::PidParameter pidParameters);
    PidWrapper::RatePidState getRatePidState();
    
  private:
    Sensors& _sensors;
    NavigationController& _navigationController;
    ConfigFileWrapper& _configFileWrapper;
    PidWrapper _yawRatePidController;
    PidWrapper _pitchRatePidController;
    PidWrapper _rollRatePidController;
    NavigationController::SetPoint _setPoints;
    Imu::Rate _rate;
    Imu::Angle _angle;
    PidWrapper::PidOutput _pidOutputs;
};

#endif