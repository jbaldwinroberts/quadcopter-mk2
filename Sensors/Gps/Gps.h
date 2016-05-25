#include "mbed.h"
#include "Global.h"
#include <TinyGPS.h>
#include "MODSERIAL.h"

#ifndef Gps_H
#define Gps_H

class Gps                
{
  public:             
    Gps(PinName gpsPinTx, PinName gpsPinRx);    
    ~Gps();
    
    struct Value
    {
        double latitude;
        double longitude;
        double altitude;
        bool fix;
    };
    
    struct Difference
    {
        double x;
        double y;
        double bearing;
        double distance;
    };
    
    Value getValues();
    Difference getDifference(Value latLong1, Value latLong2);
    
  private:
    MODSERIAL *_gps;
    TinyGPS _tinyGPS;
    Value _values;
    double deg2rad(double deg);
    double rad2deg(double rad);
};

#endif