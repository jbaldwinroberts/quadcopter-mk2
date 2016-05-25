#include "StatusLights.h"

StatusLights::StatusLights()
{   
    _ledState = 0;
    _led1 = new DigitalOut(LED1);
    _led2 = new DigitalOut(LED2);
    _led3 = new DigitalOut(LED3);
    _led4 = new DigitalOut(LED4);

    DEBUG("Status lights initialised\r\n");
}

StatusLights::~StatusLights(){}

void StatusLights::preFlight()
{
    _led1 -> write(!_led1 -> read());
}

void StatusLights::standby()
{
    _led2 -> write(!_led2 -> read());
}

void StatusLights::groundReady()
{
    _led3 -> write(!_led3 -> read());
}

void StatusLights::flying()
{
    _ledState++;
    if (_ledState > 5) { _ledState = 0; }
    
    _led1 -> write(_ledState == 0);
    _led2 -> write(_ledState == 1 || _ledState == 5);
    _led3 -> write(_ledState == 2 || _ledState == 4);
    _led4 -> write(_ledState == 3);
}

void StatusLights::error()
{
    _led1 -> write(!_led1 -> read());
    _led2 -> write(!_led2 -> read());
    _led3 -> write(!_led3 -> read());
    _led4 -> write(!_led4 -> read());
}

void StatusLights::clear()
{
    _led1->write(0);   
    _led2->write(0); 
    _led3->write(0); 
    _led4->write(0); 
    _ledState = 0;
}
