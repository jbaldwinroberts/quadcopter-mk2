#include "mbed.h"
#include "Global.h"

#ifndef Kalman_H
#define Kalman_H

class Kalman                
{
  public:             
    Kalman(double q, double r, double p, double intialValue);    
    ~Kalman();
    
    struct KalmanState
    {
        double q; //process noise covariance
        double r; //measurement noise covariance
        double x; //value
        double p; //estimation error covariance
        double k; //kalman gain
    };
    
    double update(double predicted, double measurement);
    double getEstimated();
    
  private:
    KalmanState _kalmanState;

};
#endif
