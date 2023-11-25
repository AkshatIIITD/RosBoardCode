// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 1300; // For reference purposes.
const double WHEEL_RADIUS = 0.08;        // Wheel radius in meters
const double WHEEL_BASE = 0.255;           // Center of left tire to center of right tire

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
    odomOld.pose.pose.position.x = rvizClick.pose.position.x;
    odomOld.pose.pose.position.y = rvizClick.pose.position.y;
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseRecieved = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16 &leftCount)
{
    static int lastCountL = 0;
    if (leftCount.data != 0 && lastCountL != 0) {
        int leftTicks = (leftCount.data - lastCountL);
        distanceLeft = (leftTicks / TICKS_PER_REVOLUTION) * 2 * PI * WHEEL_RADIUS;
    }
    lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16 &rightCount) {
    static int lastCountR = 0;
    if (rightCount.data != 0 && lastCountR != 0) {
        int rightTicks = rightCount.data - lastCountR;
        distanceRight = (rightTicks / TICKS_PER_REVOLUTION) * 2 * PI * WHEEL_RADIUS;
    }
    lastCountR = rightCount.data;
}

void publish_quat() {
  
  tf2::Quaternion q;       
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_footprint";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
void update_odom(tf2_ros::TransformBroadcaster tf_broad) {

    // Calculate the average distance
    double cycleDistance = (distanceRight + distanceLeft) / 2;

    // Calculate the number of radians the robot has turned since the last cycle
    double cycleAngle = asin((distanceLeft - distanceRight) / WHEEL_BASE);

    // Average angle during the last cycle
    double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

    // Calculate the new pose (x, y, and theta)
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

    // Prevent lockup from a single bad cycle
    if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z)) {
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
    }

    // Make sure theta stays in the correct range
    if (odomNew.pose.pose.orientation.z > PI) {
        odomNew.pose.pose.orientation.z -= 2 * PI;
    }
    else if (odomNew.pose.pose.orientation.z < -PI) {
        odomNew.pose.pose.orientation.z += 2 * PI;
    }

    // Compute the velocity
    odomNew.header.stamp = ros::Time::now();
    odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
    odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

    // Save the pose data for the next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
    odomOld.header.stamp = odomNew.header.stamp;

    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = odomNew.header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = odomNew.pose.pose.position.x;
    odom_tf.transform.translation.y = odomNew.pose.pose.position.y;
    odom_tf.transform.translation.z = odomNew.pose.pose.position.z;

    // Set the rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    // Broadcast the transform
    tf_broad.sendTransform(odom_tf);

    // Publish the odometry message
    odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {
    // Set the data fields of the odometry message
    odomNew.header.frame_id = "odom";
    odomNew.pose.pose.position.z = 0;
    odomNew.pose.pose.orientation.x = 0;
    odomNew.pose.pose.orientation.y = 0;
    odomNew.twist.twist.linear.x = 0;
    odomNew.twist.twist.linear.y = 0;
    odomNew.twist.twist.linear.z = 0;
    odomNew.twist.twist.angular.x = 0;
    odomNew.twist.twist.angular.y = 0;
    odomNew.twist.twist.angular.z = 0;
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialTheta;

    // Launch ROS and create a node
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle node;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    // Subscribe to ROS topics
    ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);

    // Publisher of simple odom message where orientation.z is an euler angle
    odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_euler", 100);
    odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        update_odom(tf_broadcaster);
        publish_quat();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
