#include "mbed.h"
#include "Global.h"
#include "PidWrapper.h"
#include "ConfigFileWrapper.h"
#include "NavigationController.h"

#ifndef StabController_H
#define StabController_H

class StabController               
{
  public:                  
    StabController(Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper); 
    ~StabController();
    
    PidWrapper::PidOutput compute();
    PidWrapper::FlightControllerPidParameters getPidParameters();
    void reset();
    void setYawStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setPitchStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setRollStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setYawRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setPitchRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setRollRatePidParameters(PidWrapper::PidParameter pidParameters);
    PidWrapper::StabPidState getStabPidState();
    
  private:
    Sensors& _sensors;
    NavigationController& _navigationController;
    ConfigFileWrapper& _configFileWrapper;
    PidWrapper _yawRatePidController;
    PidWrapper _pitchRatePidController;
    PidWrapper _rollRatePidController;
    PidWrapper _yawStabPidController;
    PidWrapper _pitchStabPidController;
    PidWrapper _rollStabPidController;
    NavigationController::SetPoint _setPoints;
    Imu::Rate _rate;
    Imu::Angle _angle;
    PidWrapper::PidOutput _stabPidOutputs;
    PidWrapper::PidOutput _ratePidOutputs;
};

#endif