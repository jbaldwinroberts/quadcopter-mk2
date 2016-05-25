#include "Kalman.h"

Kalman::Kalman(double q, double r, double p, double intialValue)
{
    _kalmanState = KalmanState();
    _kalmanState.q = q;
    _kalmanState.r = r;
    _kalmanState.p = p;
    _kalmanState.x = intialValue;
}

Kalman::~Kalman(){}

double Kalman::update(double predicted, double measurement)
{
  //prediction
  _kalmanState.x = predicted * _kalmanState.x;
  _kalmanState.p = _kalmanState.p + _kalmanState.q;

  //measurement
  _kalmanState.k = _kalmanState.p / (_kalmanState.p + _kalmanState.r);
  _kalmanState.x = _kalmanState.x + _kalmanState.k * (measurement - _kalmanState.x);
  _kalmanState.p = (1 - _kalmanState.k) * _kalmanState.p;
  
  return _kalmanState.x;
}

double Kalman::getEstimated()
{
    return _kalmanState.x;
}