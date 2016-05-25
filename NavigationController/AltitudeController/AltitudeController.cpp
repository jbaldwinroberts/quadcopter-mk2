#include "AltitudeController.h"

AltitudeController::AltitudeController(Sensors& sensors, ConfigFileWrapper& configFileWrapper, Status& status) : 
_sensors(sensors), _configFileWrapper(configFileWrapper), _status(status)
{
    double updateTime = (1.0 / ((float)FLIGHT_CONTROLLER_FREQUENCY / 10.0));
    _altitudeRatePidController.initialise(_configFileWrapper.getAltitudeRateParameters(), MIN_CLIMB_RATE, MAX_CLIMB_RATE, (RC_THRUST_MIN + 0.2), (RC_THRUST_MAX - 0.2), updateTime);
    _altitudeRatePidController.setBias(RC_HOVER);
    _altitudeStabPidController.initialise(_configFileWrapper.getAltitudeStabParameters(), ALTITUDE_MIN, ALTITUDE_MAX, MIN_CLIMB_RATE, MAX_CLIMB_RATE, updateTime );
    _altitudeStabPidController.setBias(RC_HOVER);
    //_setPoints = new NavigationController::SetPoint();
    
    DEBUG("Altitude controller initialised\r\n");
}

AltitudeController::~AltitudeController(){}

double AltitudeController::compute(double targetAltitude, double climbRate)
{   
    _altitude = _sensors.getAltitude();
    Imu::Velocity velocity = _sensors.getImuVelocity();
    
    float altitudeStabPidOutput = _altitudeStabPidController.compute(targetAltitude, _altitude.computed);
    
    //If pilot commanding climb rate
    if(_status.getDeadZone() == false) altitudeStabPidOutput = climbRate; //Feed to rate PID (overwriting stab PID output)
    
    float altitudeRatePidOutput = _altitudeRatePidController.compute(altitudeStabPidOutput, velocity.z);
    
    return altitudeRatePidOutput;
}

void AltitudeController::reset()
{
    _altitudeRatePidController.reset();
    _altitudeStabPidController.reset();
}

PidWrapper::NavigationControllerPidParameters AltitudeController::getPidParameters()
{
    PidWrapper::NavigationControllerPidParameters allPidParameters;
    allPidParameters.altitudeRate = _altitudeRatePidController.getPidParameters();
    allPidParameters.altitudeStab = _altitudeStabPidController.getPidParameters();
    return allPidParameters;
}

void AltitudeController::setAltitudeRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeRatePidController.setPidParameters(pidParameters);
}

void AltitudeController::setAltitudeStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeStabPidController.setPidParameters(pidParameters);
}