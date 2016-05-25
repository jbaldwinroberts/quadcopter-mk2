#ifndef NavigationController_H
#define NavigationController_H

#include "mbed.h"
#include "Global.h"
#include "Status.h"
#include "rtos.h"
#include "Rc.h"
#include "Sensors.h"
#include "AltitudeController.h"
#include "ConfigFileWrapper.h"

class NavigationController                
{
  public:             
    NavigationController(Status& status, Sensors& sensors, Rc& rc, ConfigFileWrapper& configFileWrapper);    
    ~NavigationController();
    
    struct SetPoint
    {
       double yaw;
       double pitch;
       double roll;
       double throttle;
       double yawTarget;
       double yawDifference;
       double climbRate;
       double targetAltitude;
    };
    
    SetPoint getSetPoint();
    void updateYawTarget();
    void updateAltitudeTarget();
    PidWrapper::NavigationControllerPidParameters getPidParameters();
    void setAltitudeRatePidParameters(PidWrapper::PidParameter pidParameters);
    void setAltitudeStabPidParameters(PidWrapper::PidParameter pidParameters);
    
  private:
    static void threadStarter(void const *p);
    void threadWorker();
    double map(double input, double inputMin, double inputMax, double outputMin, double outputMax);   
    
    Thread* _thread;
    Status& _status;
    Sensors& _sensors;
    Rc& _rc;
    ConfigFileWrapper& _configFileWrapper;
    void saveSettings();
    SetPoint _setPoints;
    AltitudeController* _altitudeController;
    Status::State _state;
    Status::NavigationMode _navigationMode;
    Rc::MappedRc _mappedRc;
    Imu::Angle _angle;
    Sensors::Altitude _altitude;
    double _altitudeHoldPidOutput;
};

#endif