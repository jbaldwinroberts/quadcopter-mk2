#include "RateController.h"

RateController::RateController(Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper) : _sensors(sensors), _navigationController(navigationController), _configFileWrapper(configFileWrapper)
{
    float updateTime = 1.0 / (float)FLIGHT_CONTROLLER_FREQUENCY;
    _yawRatePidController.initialise(_configFileWrapper.getYawRateParameters(), IMU_YAW_RATE_MIN, IMU_YAW_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    _pitchRatePidController.initialise(_configFileWrapper.getPitchRateParameters(), IMU_PITCH_RATE_MIN, IMU_PITCH_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    _rollRatePidController.initialise(_configFileWrapper.getRollRateParameters(), IMU_ROLL_RATE_MIN, IMU_ROLL_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    
    DEBUG("Rate controller initialised\r\n");
}

RateController::~RateController(){}

PidWrapper::PidOutput RateController::compute()
{   
    _setPoints = _navigationController.getSetPoint();
    _rate = _sensors.getRate();
    
    _pidOutputs.yaw = _yawRatePidController.compute(_setPoints.yaw, _rate.yaw);
    _pidOutputs.pitch = _pitchRatePidController.compute(_setPoints.pitch, _rate.pitch);
    _pidOutputs.roll = _rollRatePidController.compute(_setPoints.roll, _rate.roll);

    return _pidOutputs;
}

void RateController::reset()
{
    _yawRatePidController.reset();
    _pitchRatePidController.reset();
    _rollRatePidController.reset();
}


void RateController::setYawRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _yawRatePidController.setPidParameters(pidParameters);
}

void RateController::setPitchRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _pitchRatePidController.setPidParameters(pidParameters);
}

void RateController::setRollRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _rollRatePidController.setPidParameters(pidParameters);
}

PidWrapper::RatePidState RateController::getRatePidState()
{
    PidWrapper::RatePidState ratePidState;
    PidWrapper::PidState yawRatePidState;
    PidWrapper::PidState pitchRatePidState;
    PidWrapper::PidState rollRatePidState;
    
    yawRatePidState.setPoint = _setPoints.yaw;
    yawRatePidState.processValue = _rate.yaw;
    yawRatePidState.output = _pidOutputs.yaw;
    
    pitchRatePidState.setPoint = _setPoints.pitch;
    pitchRatePidState.processValue = _rate.pitch;
    pitchRatePidState.output = _pidOutputs.pitch;
    
    rollRatePidState.setPoint = _setPoints.roll;
    rollRatePidState.processValue = _rate.roll;
    rollRatePidState.output = _pidOutputs.roll;
    
    ratePidState.yawRate = yawRatePidState;
    ratePidState.pitchRate = pitchRatePidState;
    ratePidState.rollRate = rollRatePidState;
    
    return ratePidState;
}