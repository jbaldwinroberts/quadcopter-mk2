#include "mbed.h"
#include "Global.h"
#include "rtos.h"
#include "Gps.h"
#include "Imu.h"
#include "Status.h"
#include "LidarLitePwm.h"
#include "Kalman.h"
#include <math.h>

#ifndef Sensors_H
#define Sensors_H

//#define PI 3.14159265

using namespace std;

class Sensors                
{
  public:             
    Sensors(Status& status, ConfigFileWrapper& configFileWrapper, PinName gpsPinTx, PinName gpsPinRx, PinName i2cSda, PinName i2cScl, PinName lidarInterrupt);    
    ~Sensors();
    
    struct Position
    {
        Gps::Value gpsPosition;
        double computedX;
        double computedY;  
    };
    
    struct Altitude
    {
        double lidar;
        double barometer;
        double gps;
        double computed;
    };
    
    void update();
    Imu::Rate getRate();
    Imu::Angle getAngle();
    Gps::Value getGpsValues();
    Imu::Velocity getImuVelocity();
    Sensors::Altitude getAltitude();
    Sensors::Position getPosition();
    Imu::Acceleration getImuAcceleration();
    void enable(bool enable);
    void zeroBarometer();
    void updateImu();
    void zeroAccel();
    void zeroPos();
    
  private:
    void zeroGyro();
    void updateGpsValues();
    void updateAltitude();
    void updateVelocity();
    void updatePosition();
    int getLidarAltitude();
    double toRadian(double deg);
    
    Status& _status;
    Imu::Rate _rate;
    Imu::Angle _angle;
    Position _position;
    Position _startingPosition;
    Altitude _altitude;
    Gps::Value _gpsValues;
    Imu* _imu;
    Gps* _gps;
    LidarLitePwm* _lidar;
    Kalman* _altitudeKalmanFilter;
    Kalman* _xPosKalmanFilter;
    Kalman* _yPosKalmanFilter;
};

#endif