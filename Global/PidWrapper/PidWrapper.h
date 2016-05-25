#include "mbed.h"
#include "Global.h"
#include "PID.h"

#ifndef PidWrapper_H
#define PidWrapper_H

class PidWrapper                   // begin declaration of the class
{
  public:                    // begin public section
    PidWrapper();    // constructor
    ~PidWrapper();
    
    struct PidOutput
    {
        double yaw;
        double pitch;
        double roll;
    };
    
    struct PidParameter
    {
        double p;
        double i;
        double d;
    };
    
    struct PidState
    {
        double setPoint;
        double processValue;
        double output;
    };
    
    struct FlightControllerPidParameters
    {
        PidParameter yawRate;
        PidParameter pitchRate;
        PidParameter rollRate;
        PidParameter yawStab;
        PidParameter pitchStab;
        PidParameter rollStab;
    };
    
    struct NavigationControllerPidParameters
    {
        PidParameter altitudeRate;
        PidParameter altitudeStab;
    };
    
    struct RatePidState
    {
        PidState yawRate;
        PidState pitchRate;
        PidState rollRate;
    };
    
    struct StabPidState
    {
        PidState yawRate;
        PidState pitchRate;
        PidState rollRate;
        PidState yawStab;
        PidState pitchStab;
        PidState rollStab;
    };
    
    bool initialise(PidParameter pidParameter, double inputMin, double inputMax, double outputMin, double outputMax, float updateTime);
    double compute(double setPoint, double processValue);
    PidWrapper::PidParameter getPidParameters();
    void reset();
    void setPidParameters(PidWrapper::PidParameter pidParameters);
    void setBias(float bias);

  private:
    PID* _pid;
};

#endif