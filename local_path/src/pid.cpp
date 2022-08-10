#include "pid.h"

pid::pid()
{
    ros::NodeHandle pnh("~");
    pnh.param("STEER_KP",STEER_KP,1.0);
    pnh.param("STEER_KD",STEER_KD,0.0);
}

void pid::nomalize_angle(double &angle)
{
    while(angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while(angle < M_PI)
    {
        angle += 2.0 * M_PI;
    }
}

double pid::PID(double desire, double rate)
{
    double steer_error, steering_out;

    steer_error = desire;
    double steer_derivative = (steer_error - prev_error)*rate;
    steering_out = STEER_KP * steer_error + STEER_KD *steer_derivative; //+ STEER_KI*steer_integral;

    if (steering_out >= 0.35)
        steering_out = 0.35;
    if (steering_out <= -0.35)
        steering_out = -0.35;

    prev_error = steer_error;

    return steering_out;
}
