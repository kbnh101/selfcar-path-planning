#include<ros/ros.h>
#include<iostream>

class pid{
    double STEER_KP, STEER_KD, prev_error;
public:
    pid();
    //Function
    void nomalize_angle(double &angle);
    double PID(double desire, double rate);
};
