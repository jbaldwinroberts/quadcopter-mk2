#include "mbed.h"

#ifndef GLOBAL_H
#define GLOBAL_H

#define DEBUG(...) printf(__VA_ARGS__)
//#define DEBUG(a) (void)0


#define MOTORS_ENABLED

#define MOD(a) ((a > 180.0) ? (a - 360.0) : ((a < -180.0) ? (a + 360.0) : a))
#define PI 3.14159265

#define             IMU_YAW_ANGLE_MAX 180
#define             IMU_YAW_ANGLE_MIN -180
#define             IMU_ROLL_ANGLE_MAX 90
#define             IMU_ROLL_ANGLE_MIN -90
#define             IMU_PITCH_ANGLE_MAX 90
#define             IMU_PITCH_ANGLE_MIN -90
#define             IMU_YAW_RATE_MAX 360
#define             IMU_YAW_RATE_MIN -360
#define             IMU_ROLL_RATE_MAX 360
#define             IMU_ROLL_RATE_MIN -360
#define             IMU_PITCH_RATE_MAX 360
#define             IMU_PITCH_RATE_MIN -360

#define             RC_CHANNELS 8
#define             RC_THROTTLE_CHANNEL 3
#define             RC_IN_MAX 1900
#define             RC_IN_MIN 1000
#define             RC_OUT_MAX 1
#define             RC_OUT_MIN 0
#define             RC_YAW_RATE_MAX 180
#define             RC_YAW_RATE_MIN -180
#define             RC_ROLL_RATE_MAX 90
#define             RC_ROLL_RATE_MIN -90
#define             RC_PITCH_RATE_MAX 90
#define             RC_PITCH_RATE_MIN -90
#define             RC_ROLL_ANGLE_MAX 45
#define             RC_ROLL_ANGLE_MIN -45
#define             RC_PITCH_ANGLE_MAX 45
#define             RC_PITCH_ANGLE_MIN -45
#define             RC_THRUST_MAX 1
#define             RC_THRUST_MIN 0
#define             RC_DEAD_ZONE 0.1
#define             RC_HOVER 0.6

#define             MOTORS_OFF 0
#define             MOTORS_ARMED 1060
#define             MOTORS_MIN 1060
#define             MOTORS_MAX 1860

#define             RATE_PID_CONTROLLER_OUTPUT_MAX 100
#define             RATE_PID_CONTROLLER_OUTPUT_MIN -100

#define             ALTITUDE_MIN 0
#define             ALTITUDE_MAX 1000
#define             MAX_CLIMB_RATE 2.5
#define             MIN_CLIMB_RATE -2.5

#define             FLIGHT_CONTROLLER_FREQUENCY 500

#endif