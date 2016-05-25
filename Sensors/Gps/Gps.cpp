#include "Gps.h"

Gps::Gps(PinName gpsPinTx, PinName gpsPinRx)
{
    _values = Value();
    _gps = new MODSERIAL(gpsPinTx, gpsPinRx);
    _gps->baud(115200);

    DEBUG("GPS initialised\r\n");
}

Gps::~Gps(){}

Gps::Value Gps::getValues()
{
    while(_gps->readable() > 0)
    {
        int c = _gps->getc();
        if(_tinyGPS.encode(c))
        {
            unsigned long fix_age;
            _tinyGPS.f_get_position(&_values.latitude, &_values.longitude, &fix_age);
          
            _values.altitude = _tinyGPS.f_altitude();
            
            if ((fix_age == TinyGPS::GPS_INVALID_AGE) || (fix_age > 5000)) _values.fix = false;
            else _values.fix = true;
        }
    }

    return _values;
}

Gps::Difference Gps::getDifference(Value latLong1, Value latLong2)
{
    int R = 637100000; //cm
    float dLat = deg2rad(latLong2.latitude - latLong1.latitude);
    float dLon = deg2rad(latLong2.longitude - latLong1.longitude);
    latLong1.latitude = deg2rad(latLong1.latitude);
    latLong2.latitude = deg2rad(latLong2.latitude);
    
    float a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(latLong1.latitude) * cos(latLong2.latitude); 
    float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
    float d = R * c;
    
    float y = sin(dLon) * cos(latLong2.latitude);
    float x = cos(latLong1.latitude) * sin(latLong2.latitude) - sin(latLong1.latitude) * cos(latLong2.latitude) * cos(dLat);
    float brng = rad2deg(atan2(y, x));
    
    float oppAng = 90 - brng;
    float xDiff = d * sin(deg2rad(brng));
    float yDiff = d * sin(deg2rad(oppAng));
    
    //printf("d %f, opp %f, diff %f \r\n", d, oppAng, yDiff);
    //printf("lat1 %f long1 %f lat2 %f long2 %f ", latLong1.latitude, latLong1.longitude, latLong2.latitude, latLong2.longitude);
    //printf(" Bearing %f", brng);
    //printf(" Distance %f", d);
    //printf(" X %f, Y %f\r\n", xDiff, yDiff);   
    
    Difference difference = Difference();
    difference.x = xDiff;
    difference.y = yDiff;
    difference.bearing = brng;
    difference.distance = d;
    
    return difference;  
}

double Gps::deg2rad(double deg)
{
    return (deg * PI / 180.0);
}

double Gps::rad2deg(double rad) 
{
    return (rad * 180.0 / PI);
}