#include "BaseStation.h"

BaseStation::BaseStation(Status& status, Rc& rc, Sensors& sensors, NavigationController& navigationController, FlightController& flightController, ConfigFileWrapper& configFileWrapper, PinName wirelessPinTx, PinName wirelessPinRx) : _status(status), _rc(rc), _sensors(sensors), _navigationController(navigationController), _flightController(flightController), _configFileWrapper(configFileWrapper)
{
    _wireless = new MODSERIAL(wirelessPinTx, wirelessPinRx);
    _wireless->baud(57600);
    _wirelessSerialRxPos = 0;
    
    _thread = new Thread(&BaseStation::threadStarter, this, osPriorityNormal);
    DEBUG("Base Station initialised\r\n");
}

BaseStation::~BaseStation(){}
l
void BaseStation::threadStarter(void const *p)
{
    BaseStation *instance = (BaseStation*)p;
    instance->threadWorker();
}

void BaseStation::threadWorker()
{   
    while(true)
    {
        //Check comms mode and print correct data back to PC application
        Status::BaseStationMode mode = _status.getBaseStationMode();
        
        if(mode == Status::MOTOR_POWER)
        {
            MotorMixer::MotorPower motorPower = _flightController.getMotorPower();
            
            _wireless->printf("<M1=%1.2f:M2=%1.2f:M3=%1.2f:M4=%1.2f>",
            motorPower.motor1, motorPower.motor2, motorPower.motor3, motorPower.motor4);
        }
        else if (mode == Status::PID_OUTPUTS)
        {
            PidWrapper::PidOutput pidOutputs = _flightController.getPidOutputs();
            
            _wireless->printf("<YPID=%1.2f:PPID=%1.2f:RPID=%1.2f>",
            pidOutputs.yaw, pidOutputs.pitch, pidOutputs.roll);
        }
        else if (mode == Status::IMU_OUTPUTS)
        {
            Imu::Rate rate = _sensors.getRate();
            Imu::Angle angle = _sensors.getAngle();
            
            _wireless->printf("<SY=%1.2f:SP=%1.2f:SR=%1.2f:RY=%1.2f:RP=%1.2f:RR=%1.2f>",
            angle.yaw, angle.pitch, angle.roll, rate.yaw, rate.pitch, rate.roll);
        }
        else if (mode == Status::STATUS)
        {                        
            _wireless->printf("<Batt=%f:Armed=%d:Init=%d:FlightMode=%d:State=%d:NavMode=%d>",
            _status.getBatteryLevel(), _status.getArmed(), _status.getInitialised(), _status.getFlightMode(), _status.getState(), _status.getNavigationMode());
        }
        else if (mode == Status::RC)
        {
            Rc::MappedRc mappedRc = _rc.getMappedRc();
            Rc::RawRc rawRc = _rc.getRawRc();
            
            _wireless->printf("<MRCY=%1.2f:MRCP=%1.2f:MRCR=%1.2f:MRCT=%1.2f:RRC1=%1.2f:RRC2=%1.2f:RRC3=%1.2f:RRC4=%1.2f:RRC5=%1.2f:RRC6=%1.2f:RRC7=%1.2f:RRC8=%1.2f>",
            mappedRc.yaw, mappedRc.pitch, mappedRc.roll, mappedRc.throttle, rawRc.channel0, rawRc.channel1, rawRc.channel2, rawRc.channel3, rawRc.channel4, rawRc.channel5, rawRc.channel6, rawRc.channel7);
        }
        else if (mode == Status::PID_TUNING)
        {
            PidWrapper::FlightControllerPidParameters flightControllerPidParameters = _flightController.getPidParameters();
            PidWrapper::NavigationControllerPidParameters navigationControllerPidParameters = _navigationController.getPidParameters();
            _wireless->printf("<RYPIDP=%1.8f:RYPIDI=%1.8f:RYPIDD=%1.8f:RPPIDP=%1.8f:RPPIDI=%1.8f:RPPIDD=%1.8f:RRPIDP=%1.8f:RRPIDI=%1.8f:RRPIDD=%1.8f:SYPIDP=%1.8f:SYPIDI=%1.8f:SYPIDD=%1.8f:SPPIDP=%1.8f:SPPIDI=%1.8f:SPPIDD=%1.8f:SRPIDP=%1.8f:SRPIDI=%1.8f:SRPIDD=%1.8f:ARPIDP=%1.8f:ARPIDI=%1.8f:ARPIDD=%1.8f:ASPIDP=%1.8f:ASPIDI=%1.8f:ASPIDD=%1.8f>",
            flightControllerPidParameters.yawRate.p, flightControllerPidParameters.yawRate.i, flightControllerPidParameters.yawRate.d, 
            flightControllerPidParameters.pitchRate.p, flightControllerPidParameters.pitchRate.i, flightControllerPidParameters.pitchRate.d, 
            flightControllerPidParameters.rollRate.p, flightControllerPidParameters.rollRate.i, flightControllerPidParameters.rollRate.d, 
            flightControllerPidParameters.yawStab.p, flightControllerPidParameters.yawStab.i, flightControllerPidParameters.yawStab.d, 
            flightControllerPidParameters.pitchStab.p, flightControllerPidParameters.pitchStab.i, flightControllerPidParameters.pitchStab.d, 
            flightControllerPidParameters.rollStab.p, flightControllerPidParameters.rollStab.i, flightControllerPidParameters.rollStab.d,
            navigationControllerPidParameters.altitudeRate.p, navigationControllerPidParameters.altitudeRate.i, navigationControllerPidParameters.altitudeRate.d,
            navigationControllerPidParameters.altitudeStab.p, navigationControllerPidParameters.altitudeStab.i, navigationControllerPidParameters.altitudeStab.d);
        }
        else if (mode == Status::GPS)
        {
            Gps::Value gpsValues = _sensors.getGpsValues();
            
            _wireless->printf("<GLat=%1.6f:GLon=%1.6f:GAlt=%1.2f:GInit=%d>",
            gpsValues.latitude, gpsValues.longitude, gpsValues.altitude, gpsValues.fix);
        }/*
        else if (mode == Status::ZERO)
        {
                            _wirelessSerial.printf("<ZY=%1.6f:ZP=%1.6f:ZR=%1.6f>",
                _zeroValues[0], _zeroValues[1], _zeroValues[2]);
                break;
            
        }*/
        else if (mode == Status::RATE_TUNING)
        {
            if(_status.getFlightMode() == Status::RATE)
            {
                PidWrapper::RatePidState ratePidState = _flightController.getRatePidState();
                
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wireless->printf("<RYPIDS=%1.2f:RYPIDP=%1.2f:RYPIDO=%1.2f:RPPIDS=%1.2f:RPPIDP=%1.2f:RPPIDO=%1.2f:RRPIDS=%1.2f:RRPIDP=%1.2f:RRPIDO=%1.2f>",
                ratePidState.yawRate.setPoint, ratePidState.yawRate.processValue, ratePidState.yawRate.output, ratePidState.pitchRate.setPoint, ratePidState.pitchRate.processValue, ratePidState.pitchRate.output, ratePidState.rollRate.setPoint, ratePidState.rollRate.processValue, ratePidState.rollRate.output);
            }
            else if (_status.getFlightMode() == Status::STAB)
            {
                PidWrapper::StabPidState stabPidState = _flightController.getStabPidState();
                
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wireless->printf("<RYPIDS=%1.2f:RYPIDP=%1.2f:RYPIDO=%1.2f:RPPIDS=%1.2f:RPPIDP=%1.2f:RPPIDO=%1.2f:RRPIDS=%1.2f:RRPIDP=%1.2f:RRPIDO=%1.2f>",
                stabPidState.yawRate.setPoint, stabPidState.yawRate.processValue, stabPidState.yawRate.output, stabPidState.pitchRate.setPoint, stabPidState.pitchRate.processValue, stabPidState.pitchRate.output, stabPidState.rollRate.setPoint, stabPidState.rollRate.processValue, stabPidState.rollRate.output);
            }   
        }
        else if (mode == Status::STAB_TUNING)
        {
            if(_status.getFlightMode() == Status::RATE)
            {
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wireless->printf("<SYPIDS=0:SYPIDP=0:SYPIDO=0:SPPIDS=0:SPPIDP=0:SPPIDO=0:SRPIDS=0:SRPIDP=0:SRPIDO=0>");
            }
            else if (_status.getFlightMode() == Status::STAB)
            {
                PidWrapper::StabPidState stabPidState = _flightController.getStabPidState();
                
                //Yaw set point, Yaw actual, Yaw PID output
                //Pitch set point, Pitch actual, Pitch PID output
                //Roll set point, Roll actual, Roll PID output
                _wireless->printf("<SYPIDS=%1.2f:SYPIDP=%1.2f:SYPIDO=%1.2f:SPPIDS=%1.2f:SPPIDP=%1.2f:SPPIDO=%1.2f:SRPIDS=%1.2f:SRPIDP=%1.2f:SRPIDO=%1.2f>",
                stabPidState.yawStab.setPoint, stabPidState.yawStab.processValue, stabPidState.yawStab.output, stabPidState.pitchStab.setPoint, stabPidState.pitchStab.processValue, stabPidState.pitchStab.output, stabPidState.rollStab.setPoint, stabPidState.rollStab.processValue, stabPidState.rollStab.output);
            } 
        }
        else if (mode == Status::ALTITUDE)
        {
            Sensors::Altitude altitude = _sensors.getAltitude();      
            Imu::Acceleration acceleration = _sensors.getImuAcceleration();
                  
            _wireless->printf("<ZVEL=%1.2f:CAlt=%1.2f:BAlt=%1.2f:LAlt=%1.2f>",
            acceleration.z, altitude.computed, altitude.barometer, altitude.lidar);
        }
        else if (mode == Status::VELOCITY)
        {
            //Imu::Velocity velocity = _sensors.getImuVelocity();
            //_wireless->printf("<XVEL=%1.2f:YVEL=%1.2f:ZVEL=%1.2f>",
            //velocity.x, velocity.y, velocity.z);  
            Imu::Acceleration acceleration = _sensors.getImuAcceleration();
            _wireless->printf("<XVEL=%1.2f:YVEL=%1.2f:ZVEL=%1.2f>",
            acceleration.x, acceleration.y, acceleration.z);   
        }
        else if (mode == Status::ALTITUDE_STATUS)
        {
            NavigationController::SetPoint setPoints = _navigationController.getSetPoint();
            Sensors::Altitude altitude = _sensors.getAltitude();
            _wireless->printf("<ACR=%1.2f:ATA=%1.2f:ATHR=%1.2f:CAlt=%1.2f:ZVEL=%1.2f>",
            setPoints.climbRate, setPoints.targetAltitude, (setPoints.throttle * 100), altitude.computed, _sensors.getImuVelocity().z);      
        }
        else if (mode == Status::LIDAR)
        {
            Sensors::Altitude altitude = _sensors.getAltitude();
            Imu::Angle angle = _sensors.getAngle();         
            _wireless->printf("<SP=%1.2f:SR=%1.2f:LAlt=%1.2f>", angle.pitch, angle.roll, altitude.lidar);   
        }
        
        //Check for wireless serial command
        while (_wireless->readable() > 0)
        {
            int c = _wireless->getc();
                                                
            switch (c)
            {
                case 60: // 
                    _wirelessSerialRxPos = 0;
                    break;
                
                case 10: // LF
                case 13: // CR
                case 62: // >
                    checkCommand();
                    break;
                    
                default:
                    _wirelessSerialBuffer[_wirelessSerialRxPos++] = c;
                    if (_wirelessSerialRxPos > 200)
                    {
                        _wirelessSerialRxPos = 0;
                    }
                    break;
            }
        }
        
        Thread::wait(200);
    }
}

void BaseStation::checkCommand()
{
    int length = _wirelessSerialRxPos;
    _wirelessSerialBuffer[_wirelessSerialRxPos] = 0;
    _wirelessSerialRxPos = 0;

    if (length < 1)
    {
        return;
    }
    
    char command = _wirelessSerialBuffer[0];
    double value = 0;
    if(length > 1)
    {
        value = atof((char*)&_wirelessSerialBuffer[2]);
    }
    
    PidWrapper::FlightControllerPidParameters flightControllerPidParameters = _flightController.getPidParameters();
    PidWrapper::NavigationControllerPidParameters navigationControllerPidParameters = _navigationController.getPidParameters();
    
    switch (command)
    {
        case 'a':
            navigationControllerPidParameters.altitudeRate.p = value;
            _navigationController.setAltitudeRatePidParameters(navigationControllerPidParameters.altitudeRate);
            break;
            
        case 'b':
            navigationControllerPidParameters.altitudeRate.i = value;
            _navigationController.setAltitudeRatePidParameters(navigationControllerPidParameters.altitudeRate);
            break;
            
        case 'c':
            navigationControllerPidParameters.altitudeRate.d = value;
            _navigationController.setAltitudeRatePidParameters(navigationControllerPidParameters.altitudeRate);
            break;
            
        case 'd':
            navigationControllerPidParameters.altitudeStab.p = value;
            _navigationController.setAltitudeStabPidParameters(navigationControllerPidParameters.altitudeStab);
            break;
            
        case 'e':
            navigationControllerPidParameters.altitudeStab.i = value;
            _navigationController.setAltitudeStabPidParameters(navigationControllerPidParameters.altitudeStab);
            break;
            
        case 'f':
            navigationControllerPidParameters.altitudeStab.d = value;
            _navigationController.setAltitudeStabPidParameters(navigationControllerPidParameters.altitudeStab);
            break;
            
        case 'g':
            _sensors.zeroAccel();
            break;
            
        //Set PID values
        case 'h':
            flightControllerPidParameters.yawRate.p = value;
            _flightController.setYawRatePidParameters(flightControllerPidParameters.yawRate);
            break;
            
        case 'i':
            flightControllerPidParameters.yawRate.i = value;
            _flightController.setYawRatePidParameters(flightControllerPidParameters.yawRate);
            break;
            
        case 'j':
            flightControllerPidParameters.yawRate.d = value;
            _flightController.setYawRatePidParameters(flightControllerPidParameters.yawRate);
            break;
            
        case 'k':
            flightControllerPidParameters.pitchRate.p = value;
            _flightController.setPitchRatePidParameters(flightControllerPidParameters.pitchRate);
            break;
            
        case 'l':
            flightControllerPidParameters.pitchRate.i = value;
            _flightController.setPitchRatePidParameters(flightControllerPidParameters.pitchRate); 
            break;
            
        case 'm':
            flightControllerPidParameters.pitchRate.d = value;
            _flightController.setPitchRatePidParameters(flightControllerPidParameters.pitchRate);
            break;
            
        case 'n':
            flightControllerPidParameters.rollRate.p = value;
            _flightController.setRollRatePidParameters(flightControllerPidParameters.rollRate);            
            break;
            
        case 'o':
            flightControllerPidParameters.rollRate.i = value;
            _flightController.setRollRatePidParameters(flightControllerPidParameters.rollRate);    
            break;
            
        case 'p':
            flightControllerPidParameters.rollRate.d = value;
            _flightController.setRollRatePidParameters(flightControllerPidParameters.rollRate);    
            break;
            
        case 'q':
            flightControllerPidParameters.yawStab.p = value;
            _flightController.setYawStabPidParameters(flightControllerPidParameters.yawStab);    
            break;
            
        case 'r':
            flightControllerPidParameters.yawStab.i = value;
            _flightController.setYawStabPidParameters(flightControllerPidParameters.yawStab);   
            break;
            
        case 's':
            flightControllerPidParameters.yawStab.d = value;
            _flightController.setYawStabPidParameters(flightControllerPidParameters.yawStab);   
            break;
            
        case 't':
            flightControllerPidParameters.pitchStab.p = value;
            _flightController.setPitchStabPidParameters(flightControllerPidParameters.pitchStab);   
            break;
            
        case 'u':
            flightControllerPidParameters.pitchStab.i = value;
            _flightController.setPitchStabPidParameters(flightControllerPidParameters.pitchStab); 
            break;
            
        case 'v':
            flightControllerPidParameters.pitchStab.d = value;
            _flightController.setPitchStabPidParameters(flightControllerPidParameters.pitchStab); 
            break;
            
        case 'w':
            flightControllerPidParameters.rollStab.p = value;
            _flightController.setRollStabPidParameters(flightControllerPidParameters.rollStab); 
            break;
            
        case 'x':
            flightControllerPidParameters.rollStab.i = value;
            _flightController.setRollStabPidParameters(flightControllerPidParameters.rollStab);
            break;
            
        case 'y':
            flightControllerPidParameters.rollStab.d = value;
            _flightController.setRollStabPidParameters(flightControllerPidParameters.rollStab);
            break;
            
        case 'z':
            _status.setBaseStationMode(static_cast<Status::BaseStationMode>(value));
            break;
            
        default:
            break;
    }
    
    return;
}