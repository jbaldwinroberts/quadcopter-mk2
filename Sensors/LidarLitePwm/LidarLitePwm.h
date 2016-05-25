#include "mbed.h"
#include "filter.h"

#ifndef LidarLitePwm_H
#define LidarLitePwm_H
 
class LidarLitePwm
{
  public:
    LidarLitePwm(PinName input);
    ~LidarLitePwm();
    
    /// Returns range in cm as int
    int read();
    
    
    /// Returns the range in CM as an int
    operator int();
    
  private:

    /// Inturrupt at start of pulse
    void pulseStart();
    /// Interrupt at end of pulse
    void pulseStop();
    
    /// Interrupt driver for the input pin
    InterruptIn _interrupt;
    /// Timer
    Timer _timer;
    /// Time of the start of the current pulse
    int _pulseStartTime;
    /// The most recent sample
    int _range;
    
    filter* _lidarFilter;
};
 
#endif