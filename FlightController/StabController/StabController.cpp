#include "StabController.h"

StabController::StabController(Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper) : _sensors(sensors), _navigationController(navigationController), _configFileWrapper(configFileWrapper)
{
    float updateTime = 1.0 / (float)FLIGHT_CONTROLLER_FREQUENCY;
    _yawRatePidController.initialise(_configFileWrapper.getYawRateParameters(), IMU_YAW_RATE_MIN, IMU_YAW_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    _pitchRatePidController.initialise(_configFileWrapper.getPitchRateParameters(), IMU_PITCH_RATE_MIN, IMU_PITCH_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    _rollRatePidController.initialise(_configFileWrapper.getRollRateParameters(), IMU_ROLL_RATE_MIN, IMU_ROLL_RATE_MAX, RATE_PID_CONTROLLER_OUTPUT_MIN, RATE_PID_CONTROLLER_OUTPUT_MAX, updateTime);
    _yawStabPidController.initialise(_configFileWrapper.getYawStabParameters(), IMU_YAW_ANGLE_MIN, IMU_YAW_ANGLE_MAX, IMU_YAW_RATE_MIN, IMU_YAW_RATE_MAX, updateTime);
    _pitchStabPidController.initialise(_configFileWrapper.getPitchStabParameters(), IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX, IMU_PITCH_RATE_MIN, IMU_PITCH_RATE_MAX, updateTime);
    _rollStabPidController.initialise(_configFileWrapper.getRollStabParameters(), IMU_ROLL_ANGLE_MIN, IMU_ROLL_ANGLE_MAX, IMU_ROLL_RATE_MIN, IMU_ROLL_RATE_MAX, updateTime);
    
    DEBUG("Stab controller initialised\r\n");
}

StabController::~StabController(){}

PidWrapper::PidOutput StabController::compute()
{   
    _setPoints = _navigationController.getSetPoint();
    _angle = _sensors.getAngle();
    _rate = _sensors.getRate();   
        
    //_stabPidOutputs.yaw = _yawStabPidController.compute(_setPoints.yawTarget, _angle.yaw);
    _stabPidOutputs.yaw = _yawStabPidController.compute(_setPoints.yawDifference, 0);
    _stabPidOutputs.pitch = _pitchStabPidController.compute(_setPoints.pitch, _angle.pitch);
    _stabPidOutputs.roll = _rollStabPidController.compute(_setPoints.roll, _angle.roll);

    //If pilot commanding yaw
    if(abs(_setPoints.yaw) >= 5) _stabPidOutputs.yaw = _setPoints.yaw;  //Feed to rate PID (overwriting stab PID output)
    
    _ratePidOutputs.yaw = _yawRatePidController.compute(_stabPidOutputs.yaw, _rate.yaw);
    _ratePidOutputs.pitch = _pitchRatePidController.compute(_stabPidOutputs.pitch, _rate.pitch);
    _ratePidOutputs.roll = _rollRatePidController.compute(_stabPidOutputs.roll, _rate.roll);
    
    return _ratePidOutputs;
}

PidWrapper::FlightControllerPidParameters StabController::getPidParameters()
{
    PidWrapper::FlightControllerPidParameters allPidParameters;
    allPidParameters.yawStab = _yawStabPidController.getPidParameters();
    allPidParameters.pitchStab = _pitchStabPidController.getPidParameters();
    allPidParameters.rollStab = _rollStabPidController.getPidParameters();
    allPidParameters.yawRate = _yawRatePidController.getPidParameters();
    allPidParameters.pitchRate = _pitchRatePidController.getPidParameters();
    allPidParameters.rollRate = _rollRatePidController.getPidParameters();
    return allPidParameters;
}

void StabController::reset()
{
    _yawRatePidController.reset();
    _pitchRatePidController.reset();
    _rollRatePidController.reset();
    _yawStabPidController.reset();
    _pitchStabPidController.reset();
    _rollStabPidController.reset();
}

void StabController::setYawStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _yawStabPidController.setPidParameters(pidParameters);
}

void StabController::setPitchStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _pitchStabPidController.setPidParameters(pidParameters);
}

void StabController::setRollStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _rollStabPidController.setPidParameters(pidParameters);
}

void StabController::setYawRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _yawRatePidController.setPidParameters(pidParameters);
}

void StabController::setPitchRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _pitchRatePidController.setPidParameters(pidParameters);
}

void StabController::setRollRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _rollRatePidController.setPidParameters(pidParameters);
}

PidWrapper::StabPidState StabController::getStabPidState()
{
    PidWrapper::StabPidState stabPidState;
    PidWrapper::PidState yawRatePidState;
    PidWrapper::PidState pitchRatePidState;
    PidWrapper::PidState rollRatePidState;
    PidWrapper::PidState yawStabPidState;
    PidWrapper::PidState pitchStabPidState;
    PidWrapper::PidState rollStabPidState;
    
    yawRatePidState.setPoint = _stabPidOutputs.yaw;
    yawRatePidState.processValue = _rate.yaw;
    yawRatePidState.output = _ratePidOutputs.yaw;
    
    pitchRatePidState.setPoint = _stabPidOutputs.pitch;
    pitchRatePidState.processValue = _rate.pitch;
    pitchRatePidState.output = _ratePidOutputs.pitch;
    
    rollRatePidState.setPoint = _stabPidOutputs.roll;
    rollRatePidState.processValue = _rate.roll;
    rollRatePidState.output = _ratePidOutputs.roll;
    
    yawStabPidState.setPoint = _setPoints.yawDifference;
    yawStabPidState.processValue = 0;
    yawStabPidState.output = _stabPidOutputs.yaw;
    
    pitchStabPidState.setPoint = _setPoints.pitch;
    pitchStabPidState.processValue = _angle.pitch;
    pitchStabPidState.output = _stabPidOutputs.pitch;
    
    rollStabPidState.setPoint = _setPoints.roll;
    rollStabPidState.processValue = _angle.roll;
    rollStabPidState.output = _stabPidOutputs.roll;
    
    stabPidState.yawRate = yawRatePidState;
    stabPidState.pitchRate = pitchRatePidState;
    stabPidState.rollRate = rollRatePidState;
    stabPidState.yawStab = yawStabPidState;
    stabPidState.pitchStab = pitchStabPidState;
    stabPidState.rollStab = rollStabPidState;
    
    return stabPidState;
}