#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>
#include <iostream>

using namespace std;

//GPIO Pin assignments
const int leftEncoder = 23; //left encoder
const int rightEncoder = 6; //right encoder
const int leftReverse = 22; //monitor as input that goes low when left motor set to reverse
const int rightReverse = 5; //monitor as input that goes low when right motor set to reverse

//max and min allowable values
const int encoderMin = -32768;
const int encoderMax = 32768;

std_msgs::Int16 leftCount;
std_msgs::Int16 rightCount;


//this is the callback function that runs when a change of state happens on the monitored gpio pin
void left_event(int pi, unsigned int gpio, unsigned int edge, unsigned int tick)
{
if(gpio_read(pi, leftReverse)==0) //decrement if motor commanded to reverse
    {
     if(leftCount.data==encoderMin) //handle rollunder
     {
      leftCount.data = encoderMax;
     }
     else
     {
      leftCount.data--;
     }

    }
else  //increment if not commanded to reverse (must be going forward)
    {
     if(leftCount.data==encoderMax) //handle rollover
     {
      leftCount.data = encoderMin;
     }
     else
     {
      leftCount.data++;
     }

    }

}
//this is the callback function that runs when a change of state happens on the monitored gpio pin
void right_event(int pi, unsigned int gpio, unsigned int edge, unsigned int tick)
{
if(gpio_read(pi, rightReverse)==1)
    {
     if(rightCount.data==encoderMin)
     {
      rightCount.data = encoderMax;
     }
     else
     {
      rightCount.data--;
     }
    }
else
    {
     if(rightCount.data==encoderMax)
      {
       rightCount.data = encoderMin;
      }
      else
      {
       rightCount.data++;
      }
    }

}

int PigpioSetup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    //set the mode and pullup to read the encoder like a switch
    set_mode(pi, leftReverse, PI_INPUT);
    set_mode(pi, rightReverse, PI_INPUT);
    set_pull_up_down(pi, leftEncoder, PI_PUD_UP);
    set_pull_up_down(pi, rightEncoder, PI_PUD_UP);
    return pi;
}

int main(int argc, char **argv)
{
    //initialize pipiod interface
    int pi = PigpioSetup();
    if(pi>=0)
    {
        cout<<"daemon interface started ok at "<<pi<<endl;
    }
    else
    {
        cout<<"Failed to connect to PIGPIO Daemon - is it running?"<<endl;
        return -1;
    }


    //initializes callbacks
    int cbLeft = callback(pi, leftEncoder, RISING_EDGE, left_event);
    int cbRight = callback(pi, rightEncoder, RISING_EDGE, right_event);

    //normal ROS node setup: Register node with master,  advertise publishers
    ros::init(argc, argv, "tick_node");
    ros::NodeHandle node;
    ros::Publisher pubLeft = node.advertise<std_msgs::Int16>("left_ticks", 10);
    ros::Publisher pubRight = node.advertise<std_msgs::Int16>("right_ticks", 10);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        pubLeft.publish(leftCount);
        pubRight.publish(rightCount);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //terminate callbacks and pigpiod connectoin to release daemon resources
    callback_cancel(cbLeft);
    callback_cancel(cbRight);
    pigpio_stop(pi);
    return 0;
}
