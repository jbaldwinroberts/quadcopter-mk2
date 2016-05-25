#include "mbed.h"
#include "Global.h"
#include "ConfigFile.h"
#include "PidWrapper.h"

#ifndef ConfigFileWrapper_H
#define ConfigFileWrapper_H

class ConfigFileWrapper                   // begin declaration of the class
{
  public:                    // begin public section
    ConfigFileWrapper();    // constructor
    ~ConfigFileWrapper();
    
    PidWrapper::PidParameter getYawRateParameters();
    PidWrapper::PidParameter getPitchRateParameters();
    PidWrapper::PidParameter getRollRateParameters();
    PidWrapper::PidParameter getYawStabParameters();
    PidWrapper::PidParameter getPitchStabParameters();
    PidWrapper::PidParameter getRollStabParameters();
    PidWrapper::PidParameter getAltitudeRateParameters();
    PidWrapper::PidParameter getAltitudeStabParameters();
    
    bool setYawRateParameters(PidWrapper::PidParameter pidParameters);
    bool setPitchRateParameters(PidWrapper::PidParameter pidParameters);
    bool setRollRateParameters(PidWrapper::PidParameter pidParameters);
    bool setYawStabParameters(PidWrapper::PidParameter pidParameters);
    bool setPitchStabParameters(PidWrapper::PidParameter pidParameters);
    bool setRollStabParameters(PidWrapper::PidParameter pidParameters);
    bool setAltitudeRateParameters(PidWrapper::PidParameter pidParameters);
    bool setAltitudeStabParameters(PidWrapper::PidParameter pidParameters);
    
    float getAccelZeroPitch();
    float getAccelZeroRoll();
    bool setAccelZeroPitch(float value);
    bool setAccelZeroRoll(float value);
    
    bool saveSettings();
    
  private:
    ConfigFile _configFile;
    char* _str;
    PidWrapper::PidParameter _yawRateParameters;
    PidWrapper::PidParameter _pitchRateParameters;
    PidWrapper::PidParameter _rollRateParameters;
    PidWrapper::PidParameter _yawStabParameters;
    PidWrapper::PidParameter _pitchStabParameters;
    PidWrapper::PidParameter _rollStabParameters;
    PidWrapper::PidParameter _altitudeRateParameters;
    PidWrapper::PidParameter _altitudeStabParameters;
    float _accelZeroPitch;
    float _accelZeroRoll;
    void convertToCharArray(double number);
    void convertToCharArray(int number);
    void loadSettings();
};

#endif