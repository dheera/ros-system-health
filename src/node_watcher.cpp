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

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_watcher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // low importance: necessary only for autonomous driving
  std::vector<std::string> nodes_stopauto;
  std::string nodes_stopauto_str;
  pnh.param<std::string>("nodes_stopauto", nodes_stopauto_str, "/ublox_gps_node");
  if(nodes_stopauto_str != "") {
    boost::split(nodes_stopauto, nodes_stopauto_str, boost::is_any_of(","));
  }

  // medium importance: necessary for manual driving + autonomous driving
  std::vector<std::string> nodes_stopmanual;
  std::string nodes_stopmanual_str;
  pnh.param<std::string>("nodes_stopmanual", nodes_stopmanual_str, "/laserscan_avoidance_node");
  if(nodes_stopmanual_str != "") {
    boost::split(nodes_stopmanual, nodes_stopmanual_str, boost::is_any_of(","));
  }

  // high importance: necessary for any motion at all
  std::vector<std::string> nodes_stopall;
  std::string nodes_stopall_str;
  pnh.param<std::string>("nodes_stopall", nodes_stopall_str, "/arduino_imu_node,/remote_v2_node,/motor/roboteq_node");
  if(nodes_stopall_str != "") {
    boost::split(nodes_stopall, nodes_stopall_str, boost::is_any_of(","));
  }

  ros::Publisher pub_stopauto = nh.advertise<geometry_msgs::Twist>("/motion/3/cmd", 1);
  ros::Publisher pub_stopmanual = nh.advertise<geometry_msgs::Twist>("/motion/5/cmd", 1);
  ros::Publisher pub_stopall = nh.advertise<geometry_msgs::Twist>("/motion/7/cmd", 1);

  geometry_msgs::Twist msg_stop;
  msg_stop.linear.x = 0.0;
  msg_stop.angular.z = 0.0;

  ros::Rate rate(5);
  while(nh.ok()) {
    ros::spinOnce();
    rate.sleep();

    std::vector<std::string> nodes_current;
    ros::master::getNodes(nodes_current);

    for(int i=0;i<nodes_stopall.size();i++) {
      if( std::find(nodes_current.begin(), nodes_current.end(), nodes_stopall[i]) == nodes_current.end() ) {
        ROS_ERROR_STREAM_THROTTLE(3, "[stopall] dead node " << nodes_stopall[i]);
        pub_stopall.publish(msg_stop);
      }
    }

    for(int i=0;i<nodes_stopmanual.size();i++) {
      if( std::find(nodes_current.begin(), nodes_current.end(), nodes_stopmanual[i]) == nodes_current.end() ) {
        ROS_ERROR_STREAM_THROTTLE(3, "[stopmanual] dead node " << nodes_stopmanual[i]);
        pub_stopmanual.publish(msg_stop);
      }
    }

    for(int i=0;i<nodes_stopauto.size();i++) {
      if( std::find(nodes_current.begin(), nodes_current.end(), nodes_stopauto[i]) == nodes_current.end() ) {
        ROS_WARN_STREAM_THROTTLE(3, "[stopauto] dead node " << nodes_stopauto[i]);
        pub_stopauto.publish(msg_stop);
      }
    }
  }
}
