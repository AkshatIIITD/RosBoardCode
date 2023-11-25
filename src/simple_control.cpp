#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <cstdlib>

const int PWM_L = 17;
const int DIR_L = 27;
const int PWM_R = 25;
const int DIR_R = 24;

double linear = 0;
double angular = 0;
int pi = -1;

using namespace std;

void SetSpeeds(const geometry_msgs::Twist &cmdVel)
{
    linear = 400*cmdVel.linear.x;
    angular = 95*cmdVel.angular.z;
}

void SetPinValues()
{
    double leftPwmOut = linear - (angular/2);
    double rightPwmOut = linear + (angular/2);
    //set motor driver direction pins
    if(leftPwmOut > 0) {
        gpio_write(pi, DIR_L, 0);
    } else if(leftPwmOut < 0) {
        gpio_write(pi, DIR_L, 1);
    }

    if(rightPwmOut > 0) {
        gpio_write(pi, DIR_R, 1);
    } else if(rightPwmOut < 0) {
        gpio_write(pi, DIR_R, 0);
    }
    leftPwmOut = (leftPwmOut < 0 ) ? -leftPwmOut : leftPwmOut;
    rightPwmOut = (rightPwmOut < 0) ? -rightPwmOut : rightPwmOut;
    set_PWM_dutycycle(pi, PWM_L, leftPwmOut);
    set_PWM_dutycycle(pi, PWM_R, rightPwmOut);
}

int PigpioSetup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    pi = pigpio_start(addrStr, portStr);
    set_mode(pi,PWM_L, PI_OUTPUT);
    set_mode(pi,DIR_L, PI_OUTPUT);
    set_mode(pi,PWM_R, PI_OUTPUT);
    set_mode(pi,DIR_R, PI_OUTPUT);
    return pi;
}

int main(int argc, char **argv)
{
    int pi = PigpioSetup();
    if (pi >= 0)
    {
        cout << "Daemon interface started successfully at " << pi << endl;
    }
    else
    {
        cout << "Failed to connect to PIGPIO Daemon - is it running?" << endl;
        return -1;
    }
    ros::init(argc, argv, "simple_control_node");
    ros::NodeHandle node;
    ros::Subscriber subForVelocity = node.subscribe("/cmd_vel", 1, SetSpeeds, ros::TransportHints().tcpNoDelay());
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        SetPinValues();
        loop_rate.sleep();
    }
    set_PWM_dutycycle(pi, PWM_L, 0);
    set_PWM_dutycycle(pi, PWM_R, 0);
    return 0;
}
