#include "mbed.h"
#include "Global.h"
#include "FreeIMU.h"
#include "filter.h"
#include "ConfigFileWrapper.h"
#include "Kalman.h"

#ifndef Imu_H
#define Imu_H

class Imu                
{
  public:             
    Imu(ConfigFileWrapper& configFileWrapper);    
    ~Imu();
    
    struct Rate
    {
       double yaw;
       double pitch;
       double roll;
    };
    
    struct Angle
    {
       double yaw;
       double pitch;
       double roll;
    };  
    
    struct Velocity
    {
        double x;
        double y;
        double z;
    };  
    
    struct Acceleration
    {
        double x;
        double y;
        double z;
    }; 
    
    void enable(bool enable);
    
    Rate getRate();
    Angle getAngle(bool bias = true);
    Velocity getVelocity(float time);
    Velocity getVelocity();
    Acceleration getAcceleration();
    double getAltitude();
    
    void zeroGyro();
    void zeroBarometer();
    void zeroAccel();
    void setCurrentVelocity(Velocity velocity);
    
  private:
    FreeIMU _freeImu;
    filter* _barometerZeroFilter;
    filter* _barometerFilter;
    Rate _rate;
    Angle _angle;
    Velocity _velocity;
    float _barometerZero;
    ConfigFileWrapper& _configFileWrapper;
    float _accelZeroPitch;
    float _accelZeroRoll;
    Kalman* _kalmanXVelFilter;
    Kalman* _kalmanYVelFilter;
    Kalman* _kalmanZVelFilter;
};

#endif