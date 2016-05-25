#include "Status.h"

Status::Status()
{
    setFlightMode(STAB);
    setBaseStationMode(STATUS);
    setNavigationMode(NONE);
    setBatteryLevel(0);
    setRcConnected(false);
    setArmed(false);
    setMotorsSpinning(false);
    setDeadZone(false);
    setInitialised(false);
    DEBUG("Status initialised\r\n");    
}

Status::~Status(){}

bool Status::update()
{
         if(getInitialised() == false && getRcConnected() == false) setState(Status::PREFLIGHT);
    else if(getInitialised() == true && getRcConnected() == false) setState(Status::PREFLIGHT);
    else if(getInitialised() == true && getRcConnected() == true && getArmed() == false) setState(Status::STANDBY);
    else if(getInitialised() == true && getRcConnected() == true && getArmed() == true && getMotorsSpinning() == false) setState(Status::GROUND_READY);
    else if(getInitialised() == true && getRcConnected() == true && getArmed() == true && getMotorsSpinning() == true && getNavigationMode() == Status::NONE) setState(Status::MANUAL);
    else if(getInitialised() == true && getRcConnected() == true && getArmed() == true && getMotorsSpinning() == true && getNavigationMode() == Status::POSITION_HOLD) setState(Status::AUTO);
    else if(getInitialised() == true && getRcConnected() == true && getArmed() == true && getMotorsSpinning() == true && getNavigationMode() == Status::ALTITUDE_HOLD) setState(Status::STABILISED);

    //else setState(Status::ERROR); 
    
    return true;
}

bool Status::setState(State state)
{
    switch(state)
    {
        case PREFLIGHT:
            if(_state != Status::PREFLIGHT)
            {
                _state = PREFLIGHT;
                _statusLights.clear();
                DEBUG("State set to PREFLIGHT\r\n");
                return true;
            }
            _statusLights.preFlight();
            return true;
              
        case STANDBY:
            if(_state != Status::STANDBY)
            {
                _state = STANDBY;
                _statusLights.clear();
                DEBUG("State set to STANDBY\r\n");
                return true;
            }
            _statusLights.standby();
            return true;
            
        case GROUND_READY:
            if(_state != Status::GROUND_READY)
            {
                _state = GROUND_READY;
                _statusLights.clear();
                DEBUG("State set to GROUND_READY\r\n");
                return true;
            }
            _statusLights.groundReady();
            return true;
                 
        case MANUAL:
            if(_state != Status::MANUAL)
            {
                _state = MANUAL;
                _statusLights.clear();
                DEBUG("State set to MANUAL\r\n");
                return true;   
            }
            _statusLights.flying();
            return true;              
        
        case STABILISED:
            if(_state != Status::STABILISED)
            {
                _state = STABILISED;
                _statusLights.clear();
                DEBUG("State set to STABILISED\r\n");
                return true;   
            }
            _statusLights.flying();
            return true;  
              
        
        case AUTO:
            
            return true;
              
            
        case ERROR:
            if(_state != Status::ERROR)
            {
                _state = Status::ERROR;
                _statusLights.clear();
                DEBUG("State set to ERROR\r\n");
                return true;
            }
            _statusLights.error();
            return true;
              
        default:
            
            return false;
              
    }    
}

Status::State Status::getState()
{
    return _state;    
}

bool Status::setFlightMode(FlightMode flightMode)
{
    if(flightMode != _flightMode)
    {
        _flightMode = flightMode;
        DEBUG("Flight mode set to %d\r\n", _flightMode);
        return true;
    }
    else return false;
}

Status::FlightMode Status::getFlightMode()
{
    return _flightMode;
}

bool Status::setNavigationMode(NavigationMode navigationMode)
{
    if(navigationMode != _navigationMode)
    {
        _navigationMode = navigationMode;
        DEBUG("Navigation mode set to %d\r\n", _navigationMode);
        return true;
    }
    else return false;
}

Status::NavigationMode Status::getNavigationMode()
{
    return _navigationMode;
}

bool Status::setBaseStationMode(BaseStationMode baseStationMode)
{
    if(baseStationMode != _baseStationMode)
    {
        _baseStationMode = baseStationMode;
        DEBUG("Base station mode set to %d\r\n", _baseStationMode);
        return true;
    }
    return false;
}

Status::BaseStationMode Status::getBaseStationMode()
{
    return _baseStationMode;
}

bool Status::setBatteryLevel(double batteryLevel)
{
    _batteryLevel = batteryLevel;
    return true;
}

double Status::getBatteryLevel()
{
    return _batteryLevel;
}

bool Status::setArmed(bool armed)
{
    if(armed != _armed)
    {
        if(armed == false)
        {
            _armed = armed;
            DEBUG("Armed set to %d\r\n", _armed);
            return true;
        }
        else if (armed == true && _navigationMode == Status::NONE && getMotorsSpinning() == false)
        {
            _armed = armed;
            DEBUG("Armed set to %d\r\n", _armed);
            return true;
        }
        else if (armed == true && _navigationMode == Status::ALTITUDE_HOLD && getMotorsSpinning() == false)
        {
            _armed = armed;
            DEBUG("Armed set to %d\r\n", _armed);
            return true;
        }
    }
    return false;
}

bool Status::getArmed()
{
    return _armed;
}

bool Status::setInitialised(bool initialised)
{
    if(initialised != _initialised)
    {
        _initialised = initialised;
        DEBUG("Initialised set to %d\r\n", _initialised);
        return true;
    }
    else return false;
}

bool Status::getInitialised()
{
    return _initialised;
}

bool Status::setRcConnected(bool rcConnected)
{
    if(rcConnected != _rcConnected)
    {
        _rcConnected = rcConnected;
        if(_rcConnected == false)
        {
            setArmed(false);
            setMotorsSpinning(false);
            setDeadZone(false);
        }
        DEBUG("Rc connected set to %d\r\n", _rcConnected);
        return true;
    }
    else return false;
}

bool Status::getRcConnected()
{
    return _rcConnected;
}

bool Status::setMotorsSpinning(bool flying)
{
    if(flying != _flying)
    {
        _flying = flying;
        DEBUG("Flying set to %d\r\n", _flying);
        return true;
    }
    else return false;
}

bool Status::getMotorsSpinning()
{
    return _flying;
}

bool Status::setDeadZone(bool deadZone)
{
    if(deadZone != _deadZone)
    {
        _deadZone = deadZone;
        DEBUG("Dead zone set to %d\r\n", _deadZone);
        return true;
    }
    else return false;
}

bool Status::getDeadZone()
{
    return _deadZone;
}