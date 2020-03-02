#include <iostream>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/message.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <topic_tools/shape_shifter.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <string>
#include "cpslib/pslib.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "system_watcher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher pub_stopauto = nh.advertise<geometry_msgs::Twist>("/motion/3/cmd", 1);
  ros::Publisher pub_stopmanual = nh.advertise<geometry_msgs::Twist>("/motion/5/cmd", 1);
  ros::Publisher pub_stopall = nh.advertise<geometry_msgs::Twist>("/motion/7/cmd", 1);

  geometry_msgs::Twist msg_stop;
  msg_stop.linear.x = 0.0;
  msg_stop.angular.z = 0.0;

  DiskUsage du;
  double du_fraction;

  ros::Rate rate(1);
  while(nh.ok()) {
    ros::spinOnce();
    rate.sleep();

    disk_usage("/", &du);
    du_fraction = (double)du.used / (double)du.total;
    
    if(du_fraction > 0.95) {
        ROS_ERROR_STREAM_THROTTLE(3, "[stopall] Disk usage over 95%");
        pub_stopall.publish(msg_stop);
    } else if(du_fraction > 0.90) {
        ROS_WARN_STREAM_THROTTLE(3, "Disk usage at " << (int)(du_fraction * 100) << "%. Robot will stop when it is at 95%.");
    }
    
  }
}
