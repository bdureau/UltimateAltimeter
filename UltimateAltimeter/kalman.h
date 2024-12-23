#ifndef _KALMAN_H
#define _KALMAN_H

extern void KalmanInit();
extern float KalmanCalc (float altitude);

class Kalman {
  public:
    Kalman();
    float KalmanCalc(float);
    void setKalmanQ(float);
    void setKalmanR(float);
    
  private:
    //Kalman Variables
    float f_1 = 1.00000; //cast as float
    float kalman_x;
    float kalman_x_last;
    float kalman_p;
    float kalman_p_last;
    float kalman_k;
    float kalman_q;
    float kalman_r;
    float kalman_x_temp;
    float kalman_p_temp;
};
#endif
