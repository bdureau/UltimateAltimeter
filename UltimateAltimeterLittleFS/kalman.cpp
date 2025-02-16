#include "kalman.h"
/*
 * 
 * Kalman filter
 * This can be used to filter data comming out from the sensors
 * 
 */

/*
 * Kalman() constructor, initialize the kalman variables.
 */  
Kalman::Kalman()
{
  // adjust the Kalman coefficients
  kalman_q=4.0001;  //filter parameters, you can play around with them
  kalman_r=.20001;  // but these values appear to be fairly optimal

  kalman_x = 0;
  kalman_p = 0;
  kalman_x_temp = 0;
  kalman_p_temp = 0;

  kalman_x_last = 0;
  kalman_p_last = 0;

}
void Kalman::setKalmanQ(float Q) {
  kalman_q = Q;
}

void Kalman::setKalmanR(float R) {
  kalman_r = R;
}
//KalmanCalc() - Calculates new Kalman values from float value "altitude"
// This will be the ASL altitude during the flight, and the AGL altitude during dumps
float Kalman::KalmanCalc (float val)
{

  //Predict kalman_x_temp, kalman_p_temp
  kalman_x_temp = kalman_x_last;
  kalman_p_temp = kalman_p_last + kalman_r;

  //Update kalman values
  kalman_k = (f_1/(kalman_p_temp + kalman_q)) * kalman_p_temp;
  kalman_x = kalman_x_temp + (kalman_k * (val - kalman_x_temp));
  kalman_p = (f_1 - kalman_k) * kalman_p_temp;

  //Save this state for next time
  kalman_x_last = kalman_x;
  kalman_p_last = kalman_p;

  //Assign current Kalman filtered altitude to working variables
  //FLOAT Kalman-filtered value value
  return kalman_x;
}  
