#include "Imu.h"

Imu::Imu(ConfigFileWrapper& configFileWrapper) :_configFileWrapper(configFileWrapper)
{
    _rate = Rate();
    _angle = Angle();
    _velocity = Velocity();
    
    _accelZeroPitch = _configFileWrapper.getAccelZeroPitch();
    _accelZeroRoll = _configFileWrapper.getAccelZeroRoll();
    
    Thread::wait(500);
    _freeImu.init(true);
    Thread::wait(500);
    
    _barometerZeroFilter = new filter(100);
    _barometerFilter = new filter(50);
    _barometerZero = 0;
    zeroBarometer();
    
    
    _kalmanXVelFilter = new Kalman(0.1, 32, 1, 0);
    _kalmanYVelFilter = new Kalman(0.1, 32, 1, 0);
    _kalmanZVelFilter = new Kalman(0.1, 32, 1, 0);
    
    DEBUG("IMU initialised\r\n");
}

Imu::~Imu(){}

Imu::Rate Imu::getRate()
{
    float rate[3];
    _freeImu.getRate(rate);

    float yaw = rate[2];
    float pitch = rate[0];
    float roll = rate[1];
    rate[0] = yaw;
    rate[1] = pitch;
    rate[2] = roll;
    
    _rate.yaw = rate[0];
    _rate.pitch = rate[1];
    _rate.roll = rate[2];
    
    return _rate;
}

Imu::Angle Imu::getAngle(bool bias)
{
    //Get raw accel data
    float values[9];
    _freeImu.getValues(values);
    
    //Update kalman filter with raw accel data
    _kalmanXVelFilter->update(1, values[0]);
    _kalmanYVelFilter->update(1, values[1]);
    _kalmanZVelFilter->update(1, values[2]);
    
    //Get angle
    float angle[3];
    _freeImu.getYawPitchRoll(angle);
    
    //Swap orientation
    float yaw = -angle[0];
    float pitch = angle[2];
    float roll = -angle[1];
    angle[0] = yaw;
    angle[1] = pitch;
    angle[2] = roll;
    
    if(bias == true)
    {
        _angle.yaw = angle[0];
        _angle.pitch = angle[1];//; - _accelZeroPitch;
        _angle.roll = angle[2];//; - _accelZeroRoll;
    }
    else
    {
        _angle.yaw = angle[0];
        _angle.pitch = angle[1];
        _angle.roll = angle[2];
    }
    
    return _angle;
}

double Imu::getAltitude()
{
    float altitude = _barometerFilter->process(_freeImu.getBaroAlt());
    float normalAltitude = (altitude - _barometerZero);
    return (normalAltitude * 100);
}

Imu::Acceleration Imu::getAcceleration()
{
    //Get raw accel data
    float values[9];
    _freeImu.getValues(values);
    
    Acceleration acceleration;
    acceleration.x = values[0];
    acceleration.y = values[1];
    acceleration.z = values[2];
    
    return acceleration;
}

Imu::Velocity Imu::getVelocity(float time)
{
    //Get quaternion
    float q[4];
    _freeImu.getQ(q);

    //Extract accelerometer data
    float acc[3];
    acc[0]= _kalmanXVelFilter->getEstimated();
    acc[1]= _kalmanYVelFilter->getEstimated();
    acc[2]= _kalmanZVelFilter->getEstimated();
    
    //Gravity compensate
    _freeImu.gravityCompensateAcc(acc, q);
    
    //Convert acceleration to velocity
    float xAcceleration = 0;
    if(acc[0] < -0.01 ||  acc[0] > 0.01) xAcceleration = acc[0] * 9.8 * 100;
    _velocity.x += xAcceleration * time;
    
    float yAcceleration = 0;
    if(acc[1] < -0.01 ||  acc[1] > 0.01) yAcceleration = acc[1] * 9.8 * 100;
    _velocity.y += yAcceleration * time;
    
    float zAcceleration = 0;
    if(acc[2] < -0.01 ||  acc[2] > 0.01) zAcceleration = acc[2] * 9.8 * 100;
    _velocity.z += zAcceleration * time;
    
    return _velocity; //cm/s/s
}

Imu::Velocity Imu::getVelocity()
{
    return _velocity; //cm/s/s
}

void Imu::zeroGyro()
{
    _freeImu.zeroGyro();
}

void Imu::setCurrentVelocity(Velocity velocity)
{
    _velocity = velocity;
}

void Imu::zeroBarometer()
{
    //DEBUG("About to start Barometer zero\r\n");
    //Thread::wait(5000);
    
    for(int i = 0; i < 1000; i++)
    {
        _barometerZero = _barometerFilter->process(_freeImu.getBaroAlt());
    }
    DEBUG("Barometer zero %f\r\n", _barometerZero);
}

void Imu::enable(bool enable)
{
    _freeImu.sample(enable);
}

void Imu::zeroAccel()
{
     Imu::Angle angle = getAngle(false);
     _accelZeroPitch = angle.pitch;
     _accelZeroRoll = angle.roll;
     //DEBUG("Zero accel, pitch %f, roll %f\r\n", _accelZeroPitch, _accelZeroRoll);
     //_configFileWrapper.setAccelZeroPitch(_accelZeroPitch);
     //_configFileWrapper.setAccelZeroRoll(_accelZeroRoll); 
     //_configFileWrapper.saveSettings();
}