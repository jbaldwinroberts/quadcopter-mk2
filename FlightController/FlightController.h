#include "mbed.h"
#include "Global.h"
#include "rtos.h"
#include "Status.h"
#include "MotorMixer.h"
#include "PidWrapper.h"
#include "RateController.h"
#include "StabController.h"
#include "ConfigFileWrapper.h"

#ifndef FlightController_H
#define FlightController_H

class FlightController               
{
  public:                  
    FlightController(Status& status, Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper, PinName motor1, PinName motor2, PinName motor3, PinName motor4); 
    ~FlightController();
    
    MotorMixer::MotorPower getMotorPower();
    PidWrapper::PidOutput getPidOutputs();    
    PidWrapper::FlightControllerPidParameters getPidParameters();
    void setYawStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setPitchStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setRollStabPidParameters(PidWrapper::PidParameter pidParameters);
    void setYawRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setPitchRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setRollRatePidParameters(PidWrapper::PidParameter pidParameters);
    PidWrapper::RatePidState getRatePidState();
    PidWrapper::StabPidState getStabPidState();
    
  private:
    static void threadStarter(void const *p);
    void threadWorker();
    RtosTimer* _rtosTimer;
    RateController* _rateController;
    StabController* _stabController;
    void saveSettings();
    Status& _status;
    Sensors& _sensors;
    NavigationController& _navigationController;
    ConfigFileWrapper& _configFileWrapper;
    MotorMixer* _motorMixer;
    
    PidWrapper::PidOutput _pidOutputs;
};

#endif