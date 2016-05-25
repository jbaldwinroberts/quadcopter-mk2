#include "mbed.h"
#include "Global.h"
#include "rtos.h"
#include "MODSERIAL.h"
#include "Rc.h"
#include "Sensors.h"
#include "Status.h"
#include "NavigationController.h"
#include "FlightController.h"

#ifndef BaseStation_H
#define BaseStation_H

class BaseStation                
{
  public:             
    BaseStation(Status& status, Rc& rc, Sensors& sensors, NavigationController& navigationController, FlightController& flightController, ConfigFileWrapper& configFileWrapper, PinName wirelessPinTx, PinName wirelessPinRx);   
    ~BaseStation();   
    
  private:
    static void threadStarter(void const *p);
    void threadWorker();
    void checkCommand();
    
    Thread* _thread;
    MODSERIAL* _wireless;
    Status& _status;
    Rc& _rc;
    Sensors& _sensors;
    NavigationController& _navigationController;
    FlightController& _flightController;
    ConfigFileWrapper& _configFileWrapper;
    char _wirelessSerialBuffer[255];
    int _wirelessSerialRxPos;
};

#endif