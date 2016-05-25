#include "Sensors.h"

Sensors::Sensors(Status& status, ConfigFileWrapper& configFileWrapper, PinName gpsPinTx, PinName gpsPinRx, PinName i2cSda, PinName i2cScl, PinName lidarInterrupt) : _status(status)
{
    _gpsValues = Gps::Value();
    _position = Position();
    _startingPosition = Position();
    _altitude = Altitude();
    
    //Initialise IMU
    _imu = new Imu(configFileWrapper);
    _imu->zeroGyro();
    
    //Initialise GPS
    _gps = new Gps(gpsPinTx, gpsPinRx);
    
    //Initialise Lidar
    _lidar = new LidarLitePwm(lidarInterrupt);
    
    
    //Initialise kalman
    _altitudeKalmanFilter = new Kalman(1, 1, 1, 0);
    //_xPosKalmanFilter = new Kalman(0.1, 500, 1, 0);
    //_yPosKalmanFilter = new Kalman(0.1, 500, 1, 0);*/
    
    DEBUG("Sensors initialised\r\n");
}

Sensors::~Sensors(){}

void Sensors::update()
{
    //Update GPS
    updateGpsValues();
    
    //Update Altitude
    updateAltitude();
    
    //Update Position
    //updatePosition();
}

Imu::Rate Sensors::getRate()
{
    return _rate;
}

Imu::Angle Sensors::getAngle()
{
    return _angle;
}

Gps::Value Sensors::getGpsValues()
{
    return _position.gpsPosition;
}

Sensors::Altitude Sensors::getAltitude()
{
    return _altitude;   
}

Sensors::Position Sensors::getPosition()
{
    return _position;   
}

Imu::Velocity Sensors::getImuVelocity()
{
    return _imu->getVelocity();
}

Imu::Acceleration Sensors::getImuAcceleration()
{
    return _imu->getAcceleration();
}

void Sensors::updateImu()
{
    _angle = _imu->getAngle();
    _rate = _imu->getRate();
}

void Sensors::updateGpsValues()
{
    _position.gpsPosition = _gps->getValues();
}

void Sensors::updateAltitude()
{
    _altitude.lidar = getLidarAltitude();  //cm
    _altitude.barometer = _imu->getAltitude(); //cm
    _altitude.gps = _gpsValues.altitude; //m
    
    //Get IMU velocity
    double time = (1.0 / ((double)FLIGHT_CONTROLLER_FREQUENCY / 10.00)); // In seconds
    Imu::Velocity velocity = _imu->getVelocity(time);
    
    //Compute predicted altitude
    double predictedAltitude = _altitude.computed + velocity.z;
    
    //Compute predicted change in altitude
    double predictedChange = 1;
    if(_altitude.computed != 0) predictedChange = predictedAltitude / _altitude.computed;
    
    //Compute estimated altitude
    float altitude = 0;
    if(_altitude.computed < 20 * 100) altitude = _altitude.lidar;
    else altitude = _altitude.barometer;
    double estimatedAltitude = _altitudeKalmanFilter->update(predictedChange, altitude);
    
    //Compute estimated velocity
    velocity.z = estimatedAltitude - _altitude.computed;
    
    //Reset IMU velocity to estimated velocity
    _imu->setCurrentVelocity(velocity);
    
    //Save new altitude
    _altitude.computed = estimatedAltitude;
    
    //printf("%1.6f, %1.6f, %1.6f\r\n", predictedAltitude, _altitude.lidar, _altitude.computed);
}


void Sensors::updatePosition()
{
    /*
    //Get IMU velocity
    Imu::Velocity velocity = _imu->getVelocity();
    
    //Compute predicted positions
    double predictedXPos = _position.computedX + velocity.x;
    double predictedYPos = _position.computedY + velocity.y;
    
    //Compute predicted change in positions
    double predictedXChange = 1;
    double predictedYChange = 1;
    if(_position.computedX != 0) predictedXChange = predictedXPos / _position.computedX;
    if(_position.computedY != 0) predictedYChange = predictedYPos / _position.computedY;
    
    //Calc difference between current and starting GPS location
    //Assume pointed north for now
    Gps::Difference difference = _gps->getDifference(_startingPosition.gpsPosition, _position.gpsPosition);
    
    //printf("x vel %f, x diff %f, ", velocity.x, difference.x);
    //printf("y vel %f, y diff %f, ", velocity.y, difference.y);
    
    //Compute estimated position
    double estimatedXPos = _xPosKalmanFilter->update(predictedXChange, difference.x);
    double estimatedYPos = _yPosKalmanFilter->update(predictedYChange, difference.y);
    
    //Compute estimated velocity
    velocity.x = estimatedXPos - _position.computedX;
    velocity.y = estimatedYPos - _position.computedY;
    
    //Reset IMU velocity to estimated velocity
    _imu->setCurrentVelocity(velocity);
    
    //Save new position
    _position.computedX = estimatedXPos;
    _position.computedY = estimatedYPos;
    
    printf("Pos X %1.6f Pos Y %1.6f\r\n", _position.computedX, _position.computedY);*/
}

void Sensors::zeroGyro()
{
    _imu->zeroGyro();
}

void Sensors::zeroBarometer()
{
    _imu->zeroBarometer();
}

void Sensors::zeroPos()
{
    _startingPosition.gpsPosition.latitude = getGpsValues().latitude;
    _startingPosition.gpsPosition.longitude = getGpsValues().longitude;
    
    //Get IMU velocity
    Imu::Velocity velocity = _imu->getVelocity();
    
    //Set x y to 0
    velocity.x = 0;
    velocity.y = 0;
 
   //Reset IMU velocity to estimated velocity
    _imu->setCurrentVelocity(velocity);   
}

void Sensors::enable(bool enable)
{
    _imu->enable(enable);
}

int Sensors::getLidarAltitude()
{
    Imu::Angle angles = getAngle();
    int rawAltitude = _lidar->read();
    
    float correctedAltitude = rawAltitude * cos(toRadian(angles.roll)) * cos(toRadian(-angles.pitch));  
    /*
    double oppAng = 0;
    double multiplier = 0;  
    int pitchAltitude = 0;
    int rollAltitude = 0;
    
    //Calulate pitch altitude
    oppAng = 90 - abs(angles.pitch);
    multiplier = sin(oppAng*PI/180);
    pitchAltitude = multiplier * rawAltitude;
    
    //Calulate roll altitude
    oppAng = 90 - abs(angles.roll);
    multiplier = sin(oppAng*PI/180);
    rollAltitude = multiplier * rawAltitude;
    
    return (pitchAltitude + rollAltitude)/ 2;*/
    return correctedAltitude;
}

void Sensors::zeroAccel()
{
    _imu->zeroAccel();
}

double Sensors::toRadian(double deg)
{
    return (deg * PI / 180);
}