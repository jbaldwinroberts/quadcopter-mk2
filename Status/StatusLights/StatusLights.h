#include "mbed.h"
#include "Global.h"
#include "rtos.h"
#include "Gps.h"
#include "Imu.h"

#ifndef StatusLights_H
#define StatusLights_H

class StatusLights                
{
  public:             
    StatusLights();    
    ~StatusLights();
    
    void clear();
    void preFlight();
    void standby();
    void groundReady();
    void flying();
    void error();
    
  private:
    int _ledState;
    DigitalOut* _led1;
    DigitalOut* _led2;
    DigitalOut* _led3;
    DigitalOut* _led4;
};

#endif