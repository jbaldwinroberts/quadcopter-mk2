#include "FlightController.h"

FlightController::FlightController(Status& status, Sensors& sensors, NavigationController& navigationController, ConfigFileWrapper& configFileWrapper, PinName motor1, PinName motor2, PinName motor3, PinName motor4)
    : _status(status), _sensors(sensors), _navigationController(navigationController), _configFileWrapper(configFileWrapper)
{
    _motorMixer = new MotorMixer(motor1, motor2, motor3, motor4);
    _rateController = new RateController(_sensors, _navigationController, _configFileWrapper);
    _stabController = new StabController(_sensors, _navigationController, _configFileWrapper);   

    _rtosTimer = new RtosTimer(&FlightController::threadStarter, osTimerPeriodic, (void*)this);
    int updateTime = (1.0 / (float)FLIGHT_CONTROLLER_FREQUENCY) * 1000;
    _rtosTimer->start(updateTime);
    
    DEBUG("Flight controller initialised\r\n");
}

FlightController::~FlightController(){}

void FlightController::threadStarter(void const *p)
{
    FlightController *instance = (FlightController*)p;
    instance->threadWorker();
}

void FlightController::threadWorker()
{   
    Status::FlightMode flightMode = _status.getFlightMode();
    
    _sensors.updateImu();
    
    if(flightMode == Status::RATE)
    {
        //Rate mode  
        _pidOutputs = _rateController->compute(); 
    }
    else if (flightMode == Status::STAB)
    {
        //Stab mode
        _pidOutputs = _stabController->compute();  
    }
      
    Status::State state = _status.getState();
    NavigationController::SetPoint setPoints = _navigationController.getSetPoint();
    
    if(state == Status::MANUAL || state == Status::STABILISED || state == Status::AUTO)  
    {
        _motorMixer->computePower(_pidOutputs, setPoints.throttle);
    }      
    else if(state == Status::GROUND_READY) 
    {
        _motorMixer->setPower(MOTORS_ARMED);
        _rateController->reset();
        _stabController->reset(); 
    }
    else if(state == Status::STANDBY)
    {
        _motorMixer->setPower(MOTORS_OFF);
        _rateController->reset();
        _stabController->reset();
    }
    //Disable motors if state is not valid
    else _motorMixer->setPower(MOTORS_OFF);
}

MotorMixer::MotorPower FlightController::getMotorPower()
{
    return _motorMixer->getMotorPower();
}

PidWrapper::PidOutput FlightController::getPidOutputs()
{
    return _pidOutputs;  
}

PidWrapper::FlightControllerPidParameters FlightController::getPidParameters()
{
    return _stabController->getPidParameters();
}

void FlightController::setYawStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setYawStabPidParameters(pidParameters);
    _configFileWrapper.setYawStabParameters(pidParameters); 
    Status::State state = _status.getState(); 
    saveSettings();
}

void FlightController::setPitchStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setPitchStabPidParameters(pidParameters);
    _configFileWrapper.setPitchStabParameters(pidParameters);
    saveSettings();
}

void FlightController::setRollStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setRollStabPidParameters(pidParameters);
    _configFileWrapper.setRollStabParameters(pidParameters);
    saveSettings();
}

void FlightController::setYawRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setYawRatePidParameters(pidParameters);
    _rateController->setYawRatePidParameters(pidParameters);
    _configFileWrapper.setYawRateParameters(pidParameters);
    saveSettings();
}

void FlightController::setPitchRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setPitchRatePidParameters(pidParameters);
    _rateController->setPitchRatePidParameters(pidParameters);
    _configFileWrapper.setPitchRateParameters(pidParameters);
    saveSettings();
}

void FlightController::setRollRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _stabController->setRollRatePidParameters(pidParameters);
    _rateController->setRollRatePidParameters(pidParameters);
    _configFileWrapper.setRollRateParameters(pidParameters);
    saveSettings();
}

void FlightController::saveSettings()
{
    Status::State state = _status.getState(); 
    if(state == Status::STANDBY || state == Status::PREFLIGHT)
    {
        _sensors.enable(false);
        _configFileWrapper.saveSettings();
        _sensors.enable(true);
    }   
}

PidWrapper::RatePidState FlightController::getRatePidState()
{
    return _rateController->getRatePidState();
}

PidWrapper::StabPidState FlightController::getStabPidState()
{
    return _stabController->getStabPidState();
}