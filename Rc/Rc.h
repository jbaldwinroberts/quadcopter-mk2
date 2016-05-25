#include "mbed.h"
#include "Global.h"
#include "Ppm.h"
#include "filter.h"
#include "Status.h"

#ifndef Rc_H
#define Rc_H

class Rc                
{
  public:             
    Rc(Status& status, PinName pin);    
    ~Rc();
    
    struct MappedRc
    {
       double yaw;
       double pitch;
       double roll;
       double throttle;
    };
    
    struct RawRc
    {
        double channel0;
        double channel1;
        double channel2;
        double channel3;
        double channel4;
        double channel5;
        double channel6;
        double channel7;    
    };
    
    MappedRc getMappedRc();
    RawRc getRawRc();
    
  private:
    double Map(double input, double inputMin, double inputMax, double outputMin, double outputMax);
    
    Ppm* _ppm;
    Status& _status;
    filter* _yawMedianFilter;
    filter* _pitchMedianFilter;
    filter* _rollMedianFilter;
    filter* _thrustMedianFilter;
    filter* _armMedianFilter;
    filter* _modeMedianFilter;
    int _notConnectedIterator;
    int _connectedIterator;
};

#endif