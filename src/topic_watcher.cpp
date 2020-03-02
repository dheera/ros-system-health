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

void cb_record_message_time(const ros::MessageEvent<topic_tools::ShapeShifter>& msg, uint64_t& message_time){
  ros::Time time = ros::Time::now();
  message_time = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "topic_watcher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // low importance: necessary only for autonomous driving
  std::vector<std::string> topics_stopauto;
  std::string topics_stopauto_str;
  pnh.param<std::string>("topics_stopauto", topics_stopauto_str, "");
  if(topics_stopauto_str != "") {
    boost::split(topics_stopauto, topics_stopauto_str, boost::is_any_of(","));
  }

  // medium importance: necessary for manual driving + autonomous driving
  std::vector<std::string> topics_stopmanual;
  std::string topics_stopmanual_str;
  pnh.param<std::string>("topics_stopmanual", topics_stopmanual_str, "");
  if(topics_stopmanual_str != "") {
    boost::split(topics_stopmanual, topics_stopmanual_str, boost::is_any_of(","));
  }

  // high importance: necessary for any motion at all
  std::vector<std::string> topics_stopall;
  std::string topics_stopall_str;
  pnh.param<std::string>("topics_stopall", topics_stopall_str, "");
  if(topics_stopall_str != "") {
    boost::split(topics_stopall, topics_stopall_str, boost::is_any_of(","));
  }

  ros::Publisher pub_stopauto = nh.advertise<geometry_msgs::Twist>("/motion/3/cmd", 1);
  ros::Publisher pub_stopmanual = nh.advertise<geometry_msgs::Twist>("/motion/5/cmd", 1);
  ros::Publisher pub_stopall = nh.advertise<geometry_msgs::Twist>("/motion/7/cmd", 1);

  uint64_t topic_message_times_stopauto[256]; // if this is a std::vector then std::ref(topic_message_times[i]) doesn't work
  uint64_t topic_message_times_stopmanual[256]; // if this is a std::vector then std::ref(topic_message_times[i]) doesn't work
  uint64_t topic_message_times_stopall[256]; // if this is a std::vector then std::ref(topic_message_times[i]) doesn't work
  std::vector<ros::Subscriber> topic_subs_stopauto;
  std::vector<ros::Subscriber> topic_subs_stopmanual;
  std::vector<ros::Subscriber> topic_subs_stopall;

  for(int i = 0; i < topics_stopauto.size(); i++) {
    topic_message_times_stopauto[i] = 0;
    ROS_INFO_STREAM("Initializing subscriber " << i << " for topic " << topics_stopauto[i]);
    ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topics_stopauto[i], 1, boost::bind(cb_record_message_time, _1, std::ref(topic_message_times_stopauto[i])));
    topic_subs_stopauto.push_back(sub);
  }

  for(int i = 0; i < topics_stopmanual.size(); i++) {
    topic_message_times_stopmanual[i] = 0;
    ROS_INFO_STREAM("Initializing subscriber " << i << " for topic " << topics_stopmanual[i]);
    ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topics_stopmanual[i], 1, boost::bind(cb_record_message_time, _1, std::ref(topic_message_times_stopmanual[i])));
    topic_subs_stopmanual.push_back(sub);
  }

  for(int i = 0; i < topics_stopall.size(); i++) {
    topic_message_times_stopall[i] = 0;
    ROS_INFO_STREAM("Initializing subscriber " << i << " for topic " << topics_stopall[i]);
    ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topics_stopall[i], 1, boost::bind(cb_record_message_time, _1, std::ref(topic_message_times_stopall[i])));
    topic_subs_stopall.push_back(sub);
  }

  geometry_msgs::Twist msg_stop;
  msg_stop.linear.x = 0.0;
  msg_stop.angular.z = 0.0;

  ros::Rate rate(5);
  while(nh.ok()) {
    ros::spinOnce();
    rate.sleep();

    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    int stopauto = 0;
    int stopmanual = 0;
    int stopall = 0;

    for(int i = 0; i < topics_stopauto.size(); i++) {
      if(t - topic_message_times_stopauto[i] > 0.5) {
        stopauto = 1;
        ROS_WARN_STREAM_THROTTLE(3, "[stopauto] " << topics_stopauto[i] << " has not published in " << int(t - topic_message_times_stopauto[i]) << "ms");
      }
    }

    for(int i = 0; i < topics_stopmanual.size(); i++) {
      if(t - topic_message_times_stopmanual[i] > 0.5) {
        stopmanual = 1;
        ROS_ERROR_STREAM_THROTTLE(3, "[stopmanual] " << topics_stopmanual[i] << " has not published in " << int(t - topic_message_times_stopmanual[i]) << "ms");
      }
    }

    for(int i = 0; i < topics_stopall.size(); i++) {
      if(t - topic_message_times_stopall[i] > 0.5) {
        stopall = 1;
        ROS_ERROR_STREAM_THROTTLE(3, "[stopall] " << topics_stopall[i] << " has not published in " << int(t - topic_message_times_stopall[i]) << "ms");
      }
    }

    if(stopauto) {
        pub_stopauto.publish(msg_stop);
    }

    if(stopmanual) {
        pub_stopmanual.publish(msg_stop);
    }

    if(stopall) {
        pub_stopall.publish(msg_stop);
    }
  }
}
