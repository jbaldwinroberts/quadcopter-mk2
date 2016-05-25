#include "mbed.h"
#include "Global.h"
#include "StatusLights.h"

#ifndef Status_H
#define Status_H

class Status                  
{
  public:                    
    Status();    
    ~Status();                  
    
    enum State
    {
        PREFLIGHT,
        STANDBY,
        GROUND_READY,
        MANUAL,
        STABILISED,
        AUTO,
        ERROR  
    };
    
    enum FlightMode
    {
        RATE,
        STAB
    };
    
    enum NavigationMode
    {
        NONE,
        ALTITUDE_HOLD,
        POSITION_HOLD
    };
    
    enum BaseStationMode
    {
        MOTOR_POWER,
        PID_OUTPUTS,
        IMU_OUTPUTS,
        STATUS,
        RC,
        PID_TUNING,
        GPS,
        ZERO,
        RATE_TUNING,
        STAB_TUNING,
        ALTITUDE,
        VELOCITY,
        ALTITUDE_STATUS,
        LIDAR
    };
    
    bool update();
    State getState();
    bool setFlightMode(FlightMode flightMode);
    FlightMode getFlightMode();
    bool setNavigationMode(NavigationMode navigationMode);
    NavigationMode getNavigationMode();
    bool setBaseStationMode(BaseStationMode baseStationMode);
    BaseStationMode getBaseStationMode();
    bool setBatteryLevel(double batteryLevel);
    double getBatteryLevel();
    bool setArmed(bool armed);
    bool getArmed();
    bool setInitialised(bool initialised);
    bool getInitialised();
    bool setRcConnected(bool rcConnected);
    bool getRcConnected();
    bool setMotorsSpinning(bool flying);
    bool getMotorsSpinning();
    bool setDeadZone(bool flying);
    bool getDeadZone();
    
  private:             
    State _state; 
    FlightMode _flightMode;
    NavigationMode _navigationMode;          
    BaseStationMode _baseStationMode;
    StatusLights _statusLights;
    bool setState(State state);
    void flash();
    double _batteryLevel;
    bool _armed;
    bool _initialised;
    bool _rcConnected;
    bool _flying;
    bool _deadZone;
};

#endif