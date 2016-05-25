#include "LidarLitePwm.h"

LidarLitePwm::LidarLitePwm(PinName pin) : _interrupt(pin)
{
    _pulseStartTime = 0;
    _range = 0;
    _lidarFilter = new filter(5);
    _timer.start();
    _interrupt.rise(this, &LidarLitePwm::pulseStart);
    _interrupt.fall(this, &LidarLitePwm::pulseStop);
}

LidarLitePwm::~LidarLitePwm(){}

int LidarLitePwm::read()
{
    //if(_range < 30) return 0;
    //else return _range - 30;
    
    return _range - 10;
}

LidarLitePwm::operator int()
{
    return read();
}

void LidarLitePwm::pulseStart()
{
    _pulseStartTime = _timer.read_us();
}

void LidarLitePwm::pulseStop()
{
    int endTime = _timer.read_us();
    if (endTime < _pulseStartTime) return; // Escape if there's been a roll over
    int range = (endTime - _pulseStartTime) / 10; // 10uS per CM
    _range = _lidarFilter->process(range);
}