#include "Rc.h"

// A class to get RC commands and convert to correct values
//Channel 0 is roll. min 1000. max 1900
//Channel 1 is pitch. min 1000. max 1900
//Channel 2 is throttle < 900 when not connected. min 1000. max 1900
//Channel 3 is yaw. min 1000. max 1900
//Channel 4 is arm. armed > 1800 else unarmed
//Channel 5 is mode. rate > 1800. stab < 1100
//Channel 6 is spare
//Channel 7 is spare

Rc::Rc(Status& status, PinName pin) : _status(status)
{
    _notConnectedIterator = 0;
    _connectedIterator = 0;
    _ppm = new Ppm(pin, RC_OUT_MIN, RC_OUT_MAX, RC_IN_MIN, RC_IN_MAX, RC_CHANNELS, RC_THROTTLE_CHANNEL);
    _yawMedianFilter = new filter(5);
    _pitchMedianFilter = new filter(5);
    _rollMedianFilter = new filter(5);
    _thrustMedianFilter = new filter(5);
    _armMedianFilter = new filter(5);
    _modeMedianFilter = new filter(5);

    DEBUG("Rc initialised\r\n"); 
}

Rc::~Rc(){}

double Rc::Map(double input, double inputMin, double inputMax, double outputMin, double outputMax)
{
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}

Rc::MappedRc Rc::getMappedRc()
{
    Rc::RawRc rawRc = getRawRc();
    Rc::MappedRc mappedRc = MappedRc();
    
    //Check if transmitter is connected
    if(rawRc.channel2 != -1)
    {
        if(_connectedIterator < 10) _connectedIterator++;
        else
        {
            _notConnectedIterator = 0;
            _status.setRcConnected(true);
            
            //Map yaw channel
            mappedRc.yaw = _yawMedianFilter->process(-Map(rawRc.channel3, RC_OUT_MIN, RC_OUT_MAX, RC_YAW_RATE_MIN, RC_YAW_RATE_MAX));
        
            //Map thust channel
            mappedRc.throttle = _thrustMedianFilter->process(Map(rawRc.channel2, RC_OUT_MIN, RC_OUT_MAX, RC_THRUST_MIN, RC_THRUST_MAX));
            
            //Manual mode
            //Altitude mode
            if(_status.getNavigationMode() == Status::ALTITUDE_HOLD)
            {
                float maxDeadZone = (float)RC_HOVER + (float)RC_DEAD_ZONE;
                float minDeadZone = (float)RC_HOVER  - (float)RC_DEAD_ZONE;
                if((minDeadZone < mappedRc.throttle) && (mappedRc.throttle <= maxDeadZone)) _status.setDeadZone(true);
                else _status.setDeadZone(false);
            }
        
            //Map arm channel.
            rawRc.channel4 = _armMedianFilter->process(rawRc.channel4);
            if(rawRc.channel4 > 0.5) _status.setArmed(true);
            else _status.setArmed(false);
            
            //Map mode channel
            rawRc.channel5 = _modeMedianFilter->process(rawRc.channel5);
            //if(rawRc.channel5 > 0.5) _status.setFlightMode(Status::RATE);
            //else _status.setFlightMode(Status::STAB);
            if(rawRc.channel5 > 0.5) _status.setNavigationMode(Status::ALTITUDE_HOLD);
            else _status.setNavigationMode(Status::NONE);
        
            //Roll and pitch mapping depends on the mode
            if(_status.getFlightMode() == Status::STAB)//Stability mode
            {
                //Roll
                mappedRc.roll = _rollMedianFilter->process(Map(rawRc.channel0, RC_OUT_MIN, RC_OUT_MAX, RC_ROLL_ANGLE_MIN, RC_ROLL_ANGLE_MAX));
                //Pitch
                mappedRc.pitch = _pitchMedianFilter->process(-Map(rawRc.channel1, RC_OUT_MIN, RC_OUT_MAX, RC_PITCH_ANGLE_MIN, RC_PITCH_ANGLE_MAX)); //Needs to be reverse
            }
            else if(_status.getFlightMode() == Status::RATE)//Rate mode
            {
                //Roll
                mappedRc.roll = _rollMedianFilter->process(Map(rawRc.channel0, RC_OUT_MIN, RC_OUT_MAX, RC_ROLL_RATE_MIN, RC_ROLL_RATE_MAX));
                //Pitch
                mappedRc.pitch = _pitchMedianFilter->process(-Map(rawRc.channel1, RC_OUT_MIN, RC_OUT_MAX, RC_PITCH_RATE_MIN, RC_PITCH_RATE_MAX)); //Needs to be reverse
            }
        }
    }
    else if(_notConnectedIterator < 10) _notConnectedIterator++;
    else
    {
        _status.setRcConnected(false);
        _notConnectedIterator = 0;
        _connectedIterator = 0;
        mappedRc.yaw = 0;
        mappedRc.pitch = 0;
        mappedRc.roll = 0;
        mappedRc.throttle = 0;
    }
    
    return mappedRc;
}

Rc::RawRc Rc::getRawRc()
{
    double rc[8] = {0,0,0,0,0,0,0,0};
    Rc::RawRc rawRc = RawRc();
        
    //Get channel data - mapped to between 0 and 1
    _ppm->GetChannelData(rc);
    
    //Put channel data into raw rc struct
    rawRc.channel0 = rc[0];
    rawRc.channel1 = rc[1];
    rawRc.channel2 = rc[2];
    rawRc.channel3 = rc[3];
    rawRc.channel4 = rc[4];
    rawRc.channel5 = rc[5];
    rawRc.channel6 = rc[6];
    rawRc.channel7 = rc[7];
    
    return rawRc;  
}