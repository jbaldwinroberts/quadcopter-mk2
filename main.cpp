#include "mbed.h"
#include "Global.h"
#include "MODSERIAL.h"
#include "Status.h"
#include "Sensors.h"
#include "BaseStation.h"
#include "Rc.h"
#include "FlightController.h"
#include "NavigationController.h"
#include "ConfigFileWrapper.h"

//Debug serial
MODSERIAL                           _debug(USBTX, USBRX);

//Unused analog pins, set to DigitalOut to remove noise.
DigitalOut                          _spare1(p16);
DigitalOut                          _spare2(p17);
DigitalOut                          _spare3(p18);
DigitalOut                          _spare4(p19);
  
int main()
{
    _debug.baud(115200);
    
    DEBUG("\r\n");  
    DEBUG("********************************************************************************\r\n");
    DEBUG("Starting Setup\r\n");
    DEBUG("********************************************************************************\r\n");
    
    ConfigFileWrapper _configFileWrapper = ConfigFileWrapper(); // No update
    Thread::wait(100); 
    Status _status = Status(); // 10 Hz called from main
    Thread::wait(100); 
    Sensors _sensors = Sensors(_status, _configFileWrapper, p13, p14, p28, p27, p15); // 50Hz called from navigation controller
    Thread::wait(100); 
    Rc _rc = Rc(_status, p8); // 50Hz called from navigation controller
    Thread::wait(100); 
    NavigationController _navigationController = NavigationController(_status, _sensors, _rc, _configFileWrapper); // 50Hz internal thread
    Thread::wait(100); 
    FlightController _flightController = FlightController(_status, _sensors, _navigationController, _configFileWrapper, p21, p22, p23, p24); // 500Hz internal thread
    Thread::wait(100); 
    BaseStation _baseStation = BaseStation(_status, _rc, _sensors, _navigationController, _flightController, _configFileWrapper, p9, p10); // 5Hz internal thread
    Thread::wait(100); 
    
    //Thread::wait(10000); 
    
    DEBUG("********************************************************************************\r\n");
    DEBUG("Finished Setup\r\n");
    DEBUG("********************************************************************************\r\n"); 
    
    _status.setInitialised(true);
    
    osThreadSetPriority(osThreadGetId(), osPriorityNormal);
    while(true)
    {
        _status.update();
        Thread::wait(100);   
    }
}
