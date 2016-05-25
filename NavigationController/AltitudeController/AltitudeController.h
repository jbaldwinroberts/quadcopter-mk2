#ifndef AltitudeController_H
#define AltitudeController_H

#include "mbed.h"
#include "Global.h"
#include "PidWrapper.h"
#include "Sensors.h"
#include "ConfigFileWrapper.h"
#include "Status.h"

class AltitudeController               
{
  public:                  
    AltitudeController(Sensors& sensors, ConfigFileWrapper& configFileWrapper, Status& status); 
    ~AltitudeController();
    
    double compute(double targetAltitude, double climbRate);
    void reset();
    void setAltitudeRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setAltitudeStabPidParameters(PidWrapper::PidParameter pidParameters);
    PidWrapper::NavigationControllerPidParameters getPidParameters();
    
  private:
    Sensors& _sensors;
    ConfigFileWrapper& _configFileWrapper;
    Status& _status;
    Sensors::Altitude _altitude;
    PidWrapper _altitudeRatePidController;
    PidWrapper _altitudeStabPidController;
};

#endif