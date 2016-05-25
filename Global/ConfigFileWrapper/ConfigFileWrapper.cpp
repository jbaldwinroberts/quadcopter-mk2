#include "ConfigFileWrapper.h"

ConfigFileWrapper::ConfigFileWrapper()
{
    _str = new char[1024];
    loadSettings();
    
    DEBUG("Config file wrapper initialised\r\n");
}

ConfigFileWrapper::~ConfigFileWrapper(){}

PidWrapper::PidParameter ConfigFileWrapper::getYawRateParameters()
{
    return _yawRateParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getPitchRateParameters()
{
    return _pitchRateParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getRollRateParameters()
{
    return _rollRateParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getYawStabParameters()
{
    return _yawStabParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getPitchStabParameters()
{
    return _pitchStabParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getRollStabParameters()
{
    return _rollStabParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getAltitudeRateParameters()
{
    return _altitudeRateParameters;
}

PidWrapper::PidParameter ConfigFileWrapper::getAltitudeStabParameters()
{
    return _altitudeStabParameters;
}

bool ConfigFileWrapper::setYawRateParameters(PidWrapper::PidParameter pidParameters)
{
    _yawRateParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setPitchRateParameters(PidWrapper::PidParameter pidParameters)
{
    _pitchRateParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setRollRateParameters(PidWrapper::PidParameter pidParameters)
{
    _rollRateParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setYawStabParameters(PidWrapper::PidParameter pidParameters)
{
    _yawStabParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setPitchStabParameters(PidWrapper::PidParameter pidParameters)
{
    _pitchStabParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setRollStabParameters(PidWrapper::PidParameter pidParameters)
{
    _rollStabParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setAltitudeRateParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeRateParameters = pidParameters;
    return true;
}

bool ConfigFileWrapper::setAltitudeStabParameters(PidWrapper::PidParameter pidParameters)
{
    _altitudeStabParameters = pidParameters;
    return true;
}

float ConfigFileWrapper::getAccelZeroPitch()
{
    return _accelZeroPitch;
}

float ConfigFileWrapper::getAccelZeroRoll()
{
    return _accelZeroRoll;
}

bool ConfigFileWrapper::setAccelZeroPitch(float value)
{
    _accelZeroPitch = value;
    return true;
}

bool ConfigFileWrapper::setAccelZeroRoll(float value)
{
    _accelZeroRoll = value;
    return true;
}

void ConfigFileWrapper::convertToCharArray(double number)
{
    sprintf(_str, "%1.8f", number );  
}

void ConfigFileWrapper::convertToCharArray(int number)
{
    sprintf(_str, "%d", number );  
}

void ConfigFileWrapper::loadSettings()
{
    char value[BUFSIZ];
    
    DEBUG("Loading settings from config file\n\r");
    
    //Read a configuration file from a mbed.
    LocalFileSystem local("local");
    if (!_configFile.read("/local/CONFIG.CFG"))
    {
        DEBUG("Config file does not exist\n\r");
        return;
    }
    else
    {    
        //Get values
        if (_configFile.getValue("yawRatePIDControllerP", &value[0], sizeof(value))) _yawRateParameters.p = atof(value);
        else DEBUG("Failed to get value for yawRatePIDControllerP\n\r");
        
        if (_configFile.getValue("yawRatePIDControllerI", &value[0], sizeof(value))) _yawRateParameters.i = atof(value);
        else DEBUG("Failed to get value for yawRatePIDControllerI\n\r");

        if (_configFile.getValue("yawRatePIDControllerD", &value[0], sizeof(value))) _yawRateParameters.d = atof(value);
        else DEBUG("Failed to get value for yawRatePIDControllerD\n\r");

        if (_configFile.getValue("pitchRatePIDControllerP", &value[0], sizeof(value))) _pitchRateParameters.p = atof(value);
        else DEBUG("Failed to get value for pitchRatePIDControllerP\n\r");

        if (_configFile.getValue("pitchRatePIDControllerI", &value[0], sizeof(value))) _pitchRateParameters.i = atof(value);
        else DEBUG("Failed to get value for pitchRatePIDControllerI\n\r");

        if (_configFile.getValue("pitchRatePIDControllerD", &value[0], sizeof(value))) _pitchRateParameters.d = atof(value);
        else DEBUG("Failed to get value for pitchRatePIDControllerD\n\r");

        if (_configFile.getValue("rollRatePIDControllerP", &value[0], sizeof(value))) _rollRateParameters.p = atof(value);
        else DEBUG("Failed to get value for rollRatePIDControllerP\n\r");

        if (_configFile.getValue("rollRatePIDControllerI", &value[0], sizeof(value))) _rollRateParameters.i = atof(value);
        else DEBUG("Failed to get value for rollRatePIDControllerI\n\r");

        if (_configFile.getValue("rollRatePIDControllerD", &value[0], sizeof(value))) _rollRateParameters.d = atof(value);
        else DEBUG("Failed to get value for rollRatePIDControllerD\n\r");

        if (_configFile.getValue("yawStabPIDControllerP", &value[0], sizeof(value))) _yawStabParameters.p = atof(value);
        else DEBUG("Failed to get value for yawStabPIDControllerP\n\r");
 
        if (_configFile.getValue("yawStabPIDControllerI", &value[0], sizeof(value))) _yawStabParameters.i = atof(value);
        else DEBUG("Failed to get value for yawStabPIDControllerI\n\r");

        if (_configFile.getValue("yawStabPIDControllerD", &value[0], sizeof(value))) _yawStabParameters.d = atof(value);
        else DEBUG("Failed to get value for yawStabPIDControllerD\n\r");

        if (_configFile.getValue("pitchStabPIDControllerP", &value[0], sizeof(value))) _pitchStabParameters.p = atof(value);
        else DEBUG("Failed to get value for pitchStabPIDControllerP\n\r");

        if (_configFile.getValue("pitchStabPIDControllerI", &value[0], sizeof(value))) _pitchStabParameters.i = atof(value);
        else DEBUG("Failed to get value for pitchStabPIDControllerI\n\r");

        if (_configFile.getValue("pitchStabPIDControllerD", &value[0], sizeof(value))) _pitchStabParameters.d = atof(value);
        else DEBUG("Failed to get value for pitchStabPIDControllerD\n\r");

        if (_configFile.getValue("rollStabPIDControllerP", &value[0], sizeof(value))) _rollStabParameters.p = atof(value);
        else DEBUG("Failed to get value for rollStabPIDControllerP\n\r");

        if (_configFile.getValue("rollStabPIDControllerI", &value[0], sizeof(value))) _rollStabParameters.i = atof(value);
        else DEBUG("Failed to get value for rollStabPIDControllerI\n\r");

        if (_configFile.getValue("rollStabPIDControllerD", &value[0], sizeof(value))) _rollStabParameters.d = atof(value);
        else DEBUG("Failed to get value for rollStabPIDControllerD\n\r");
        
        if (_configFile.getValue("altitudeRatePIDControllerP", &value[0], sizeof(value))) _altitudeRateParameters.p = atof(value);
        else DEBUG("Failed to get value for altitudeRatePIDControllerP\n\r");
        
        if (_configFile.getValue("altitudeRatePIDControllerI", &value[0], sizeof(value))) _altitudeRateParameters.i = atof(value);
        else DEBUG("Failed to get value for altitudeRatePIDControllerI\n\r");
        
        if (_configFile.getValue("altitudeRatePIDControllerD", &value[0], sizeof(value))) _altitudeRateParameters.d = atof(value);
        else DEBUG("Failed to get value for altitudeRatePIDControllerD\n\r");
        
        if (_configFile.getValue("altitudeStabPIDControllerP", &value[0], sizeof(value))) _altitudeStabParameters.p = atof(value);
        else DEBUG("Failed to get value for altitudeStabPIDControllerP\n\r");
        
        if (_configFile.getValue("altitudeStabPIDControllerI", &value[0], sizeof(value))) _altitudeStabParameters.i = atof(value);
        else DEBUG("Failed to get value for altitudeStabPIDControllerI\n\r");
        
        if (_configFile.getValue("altitudeStabPIDControllerD", &value[0], sizeof(value))) _altitudeStabParameters.d = atof(value);
        else DEBUG("Failed to get value for altitudeStabPIDControllerD\n\r");
        
        if (_configFile.getValue("accelZeroPitch", &value[0], sizeof(value))) _accelZeroPitch = atof(value);
        else DEBUG("Failed to get value for accelZeroPitch\n\r");
        
        if (_configFile.getValue("accelZeroRoll", &value[0], sizeof(value))) _accelZeroRoll = atof(value);
        else DEBUG("Failed to get value for accelZeroRoll\n\r");

        /*if (_configFile.getValue("zeroPitch", &value[0], sizeof(value))) _zeroValues[1] = atof(value);
        else DEBUG("Failed to get value for zero pitch\n\r");

        if (_configFile.getValue("zeroRoll", &value[0], sizeof(value))) _zeroValues[2] = atof(value);
        else printf("Failed to get value for zero roll\n\r");*/
    }
    
    DEBUG("Finished loading settings from config file\n\r");
}

bool ConfigFileWrapper::saveSettings()
{
    DEBUG("Writing settings to config file\n\r");
    
    LocalFileSystem local("local");
    if (!_configFile.read("/local/CONFIG.CFG"))
    {
        DEBUG("Config file does not exist\n\r");
        return false;
    }
    else
    {   
        //Write values
        convertToCharArray(_yawRateParameters.p);
        if (!_configFile.setValue("yawRatePIDControllerP", _str)) DEBUG("Failed to write value for yawRatePIDControllerP\n\r");
        
        convertToCharArray(_yawRateParameters.i);
        if (!_configFile.setValue("yawRatePIDControllerI", _str)) DEBUG("Failed to write value for yawRatePIDControllerI\n\r");
        
        convertToCharArray(_yawRateParameters.d);
        if (!_configFile.setValue("yawRatePIDControllerD", _str)) DEBUG("Failed to write value for yawRatePIDControllerD\n\r");
        
        convertToCharArray(_pitchRateParameters.p);
        if (!_configFile.setValue("pitchRatePIDControllerP", _str)) DEBUG("Failed to write value for pitchRatePIDControllerP\n\r");
        
        convertToCharArray(_pitchRateParameters.i);
        if (!_configFile.setValue("pitchRatePIDControllerI", _str)) DEBUG("Failed to write value for pitchRatePIDControllerI\n\r");
        
        convertToCharArray(_pitchRateParameters.d);
        if (!_configFile.setValue("pitchRatePIDControllerD", _str)) DEBUG("Failed to write value for pitchRatePIDControllerD\n\r");
        
        convertToCharArray(_rollRateParameters.p);
        if (!_configFile.setValue("rollRatePIDControllerP", _str)) DEBUG("Failed to write value for rollRatePIDControllerP\n\r");
        
        convertToCharArray(_rollRateParameters.i);
        if (!_configFile.setValue("rollRatePIDControllerI", _str)) DEBUG("Failed to write value for rollRatePIDControllerI\n\r");
        
        convertToCharArray(_rollRateParameters.d);
        if (!_configFile.setValue("rollRatePIDControllerD", _str)) DEBUG("Failed to write value for rollRatePIDControllerD\n\r");
    
        convertToCharArray(_yawStabParameters.p);
        if (!_configFile.setValue("yawStabPIDControllerP", _str)) DEBUG("Failed to write value for yawStabPIDControllerP\n\r");
        
        convertToCharArray(_yawStabParameters.i);
        if (!_configFile.setValue("yawStabPIDControllerI", _str)) DEBUG("Failed to write value for yawStabPIDControllerI\n\r");
        
        convertToCharArray(_yawStabParameters.d);
        if (!_configFile.setValue("yawStabPIDControllerD", _str)) DEBUG("Failed to write value for yawStabPIDControllerD\n\r");
        
        convertToCharArray(_pitchStabParameters.p);
        if (!_configFile.setValue("pitchStabPIDControllerP", _str)) DEBUG("Failed to write value for pitchStabPIDControllerP\n\r");
        
        convertToCharArray(_pitchStabParameters.i);
        if (!_configFile.setValue("pitchStabPIDControllerI", _str)) DEBUG("Failed to write value for pitchStabPIDControllerI\n\r");
        
        convertToCharArray(_pitchStabParameters.d);
        if (!_configFile.setValue("pitchStabPIDControllerD", _str)) DEBUG("Failed to write value for pitchStabPIDControllerD\n\r");
        
        convertToCharArray(_rollStabParameters.p);
        if (!_configFile.setValue("rollStabPIDControllerP", _str)) DEBUG("Failed to write value for rollStabPIDControllerP\n\r");
        
        convertToCharArray(_rollStabParameters.i);
        if (!_configFile.setValue("rollStabPIDControllerI", _str)) DEBUG("Failed to write value for _rollStabPIDControllerI\n\r");
        
        convertToCharArray(_rollStabParameters.d);
        if (!_configFile.setValue("rollStabPIDControllerD", _str)) DEBUG("Failed to write value for rollStabPIDControllerD\n\r");
        
        convertToCharArray(_altitudeRateParameters.p);
        if (!_configFile.setValue("altitudeRatePIDControllerP", _str)) DEBUG("Failed to write value for altitudeRatePIDControllerP\n\r");
        
        convertToCharArray(_altitudeRateParameters.i);
        if (!_configFile.setValue("altitudeRatePIDControllerI", _str)) DEBUG("Failed to write value for altitudeRatePIDControllerI\n\r");
        
        convertToCharArray(_altitudeRateParameters.d);
        if (!_configFile.setValue("altitudeRatePIDControllerD", _str)) DEBUG("Failed to write value for altitudeRatePIDControllerD\n\r");
        
        convertToCharArray(_altitudeStabParameters.p);
        if (!_configFile.setValue("altitudeStabPIDControllerP", _str)) DEBUG("Failed to write value for altitudeStabPIDControllerP\n\r");
        
        convertToCharArray(_altitudeStabParameters.i);
        if (!_configFile.setValue("altitudeStabPIDControllerI", _str)) DEBUG("Failed to write value for altitudeStabPIDControllerI\n\r");
        
        convertToCharArray(_altitudeStabParameters.d);
        if (!_configFile.setValue("altitudeStabPIDControllerD", _str)) DEBUG("Failed to write value for altitudeStabPIDControllerD\n\r");
        
        convertToCharArray(_accelZeroPitch);
        if (!_configFile.setValue("accelZeroPitch", _str)) DEBUG("Failed to write value for accelZeroPitch\n\r");
        
        convertToCharArray(_accelZeroRoll);
        if (!_configFile.setValue("accelZeroRoll", _str)) DEBUG("Failed to write value for accelZeroRoll\n\r");
        
    
        //convertToCharArray(_zeroValues[1]);
        //if (!_configFile.setValue("_zeroPitch", _str)) DEBUG("Failed to write value for zero pitch\n\r");
        
        //convertToCharArray(_zeroValues[2]);
        //if (!_configFile.setValue("_zeroRoll", _str)) DEBUG("Failed to write value for zero roll\n\r");
        
        if (!_configFile.write("/local/CONFIG.CFG"))
        {
            DEBUG("Failure to write settings to configuration file.\n\r");
            return false;
        }
        else
        {
            DEBUG("Successfully wrote settings to configuration file.\n\r");
            return true;
        }
    }
}
