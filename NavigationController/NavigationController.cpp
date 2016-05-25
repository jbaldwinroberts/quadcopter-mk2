#include "NavigationController.h"

NavigationController::NavigationController(Status& status, Sensors& sensors, Rc& rc, ConfigFileWrapper& configFileWrapper) : 
_status(status), _sensors(sensors), _rc(rc), _configFileWrapper(configFileWrapper)
{
    _altitudeController = new AltitudeController(_sensors, _configFileWrapper, _status);
    _altitudeHoldPidOutput = 0;
    
    _thread = new Thread(&NavigationController::threadStarter, this, osPriorityHigh);

    DEBUG("Navigation controller initialised\r\n");    
}

NavigationController::~NavigationController(){}

void NavigationController::threadStarter(void const *p)
{
    NavigationController *instance = (NavigationController*)p;
    instance->threadWorker();
}

void NavigationController::threadWorker()
{
    while(true)
    {
        //Get latest sensor readings
        _sensors.update();
        
        //Get state
        _state = _status.getState();
        
        //Get navigation mode
        _navigationMode = _status.getNavigationMode();
        
        //Get Rc commands
        _mappedRc = _rc.getMappedRc();
        
        //Get angle to calculate yaw
        _angle = _sensors.getAngle();
        
        //Reset accel data if not flying
        if(_state == Status::PREFLIGHT || _state == Status::STANDBY)
        {
            //Reset accel
            _sensors.zeroAccel();
            
            //Reset Gps
            _sensors.zeroPos();
        }
        
        //Update yaw target
        if(abs(_mappedRc.yaw) >= 5 || _state == Status::PREFLIGHT || _state == Status::STANDBY) updateYawTarget();
             
        //Make sure we are initialised         
        if(_state != Status::PREFLIGHT)
        {            
            //Update yaw difference
            _setPoints.yawDifference = MOD(_setPoints.yawTarget - _angle.yaw); 
            
            if(_navigationMode == Status::NONE)
            {
                //Motors are directly controlled by rc remote
                if(_mappedRc.throttle >= RC_DEAD_ZONE) _status.setMotorsSpinning(true);
                else _status.setMotorsSpinning(false);
                
                //Update target altitude 
                _setPoints.climbRate = 0;
                updateAltitudeTarget();
            }
            else if(_navigationMode == Status::ALTITUDE_HOLD)
            {
                //Motors are directly controlled by rc remote
                //if(_mappedRc.throttle >= RC_DEAD_ZONE) _status.setMotorsSpinning(true);
                //else _status.setMotorsSpinning(false);
                
                //Check if throttle is in dead zone
                if(_status.getDeadZone() == true) _setPoints.climbRate = 0;
                else
                {
                    //Throttle not in dead zone so map to climb rate
                    _setPoints.climbRate = map(_mappedRc.throttle, RC_THRUST_MIN, RC_THRUST_MAX, MIN_CLIMB_RATE, MAX_CLIMB_RATE);
                    
                    //float target = _setPoints.targetAltitude + (_setPoints.climbRate * 0.02);
                    
                    //Update target altitude
                    updateAltitudeTarget();
                }
                
                //If altitude is less than 10cm the directly map the rc throttle stick to the throttle
                //else use the throttle from the altitude PID controller
                if(_altitude.computed > 10) _altitudeHoldPidOutput = _altitudeController->compute(_setPoints.targetAltitude, _setPoints.climbRate);
                else _status.setMotorsSpinning(false);
            }
           /* else if(_navigationMode == Status::AUTO_LAND)
            {
                //Motors are directly controlled by rc remote
                if(_mappedRc.throttle >= RC_DEAD_ZONE) _status.setMotorsSpinning(true);
                else _status.setMotorsSpinning(false);
                
                //Get altitude
                _altitude = _sensors.getAltitude();
                
                if(_altitude.computed > 1000) _setPoints.targetAltitude = 300;
                else if(_altitude.computed < 600) _setPoints.targetAltitude = 100;
                else if(_altitude.computed < 300) _setPoints.targetAltitude = 0;
                
                //If altitude is less than 10 the directly map the rc throttle stick to the throttle
                //else use the throttle from the altitude PID controller
                if(_altitude.computed > 10) _altitudeHoldPidOutput = _altitudeController->compute(_setPoints.targetAltitude, _setPoints.climbRate);
                else _altitudeHoldPidOutput = _mappedRc.throttle;
            }*/
            
            if(_state == Status::STANDBY)  
            {
                _setPoints.yaw = 0;
                _setPoints.pitch = 0;
                _setPoints.roll = 0;
                _setPoints.throttle = 0; 
                
                _altitudeController->reset();
            }
            else if(_state == Status::GROUND_READY)  
            {
                _setPoints.yaw = _mappedRc.yaw;
                _setPoints.pitch = _mappedRc.pitch;
                _setPoints.roll = _mappedRc.roll;
                _setPoints.throttle = _mappedRc.throttle; 
                
                _altitudeController->reset();
            }
            else if(_state == Status::MANUAL)  
            {
                _setPoints.yaw = _mappedRc.yaw;
                _setPoints.pitch = _mappedRc.pitch;
                _setPoints.roll = _mappedRc.roll;
                _setPoints.throttle = _mappedRc.throttle; 
                
                _altitudeController->reset();
            }
            else if(_state == Status::STABILISED)
            {
                _setPoints.yaw = _mappedRc.yaw;
                _setPoints.pitch = _mappedRc.pitch;
                _setPoints.roll = _mappedRc.roll;
                _setPoints.throttle = _altitudeHoldPidOutput;
            }
            else if(_state == Status::AUTO)
            {
                //Waypoint navigation
            }
        }
        //Not initialised
        else
        {
            _setPoints.yaw = 0;
            _setPoints.pitch = 0;
            _setPoints.roll = 0;
            _setPoints.throttle = 0;
            
            _altitudeController->reset();
        }
        
        Thread::wait(20);
    }
}

NavigationController::SetPoint NavigationController::getSetPoint()
{
    return _setPoints;
}

void NavigationController::updateYawTarget()
{
   _setPoints.yawTarget = _angle.yaw;
}

void NavigationController::updateAltitudeTarget()
{
    _altitude = _sensors.getAltitude();
    _setPoints.targetAltitude = _altitude.computed;
    if(_setPoints.targetAltitude <= ALTITUDE_MIN) _setPoints.targetAltitude = ALTITUDE_MIN;
    else if(_setPoints.targetAltitude >= ALTITUDE_MAX) _setPoints.targetAltitude = ALTITUDE_MAX;
}
  
double NavigationController::map(double input, double inputMin, double inputMax, double outputMin, double outputMax)
{
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}  

PidWrapper::NavigationControllerPidParameters NavigationController::getPidParameters()
{
    return _altitudeController->getPidParameters();
}

void NavigationController::setAltitudeRatePidParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeController->setAltitudeRatePidParameters(pidParameters);
    _configFileWrapper.setAltitudeRateParameters(pidParameters);
    saveSettings();
}

void NavigationController::setAltitudeStabPidParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeController->setAltitudeStabPidParameters(pidParameters);
    _configFileWrapper.setAltitudeStabParameters(pidParameters);
    saveSettings();
}

void NavigationController::saveSettings()
{
    Status::State state = _status.getState(); 
    if(state == Status::STANDBY || state == Status::PREFLIGHT)
    {
        _sensors.enable(false);
        _configFileWrapper.saveSettings();
        _sensors.enable(true);
    }   
}