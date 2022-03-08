#ifndef ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H
#define ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H

#include <boost/thread/mutex.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

#include "raspicat/LightSensorValues.h"

namespace analog_distance_sensor_to_laserscan
{
class AnalogDistanceSensorToLaserscan : public nodelet::Nodelet
{
 public:
  AnalogDistanceSensorToLaserscan(){};

 private:
  ros::Publisher ls_pub_, lf_pub_, rf_pub_, rs_pub_;
  ros::Subscriber light_sensor_subscriber_;
  ros::Time initial_time_, latest_sub_time_;
  ros::Timer timer_;

  boost::mutex connect_mutex_;

  // ROS Parameters
  std::string ls_frame_id_, lf_frame_id_, rf_frame_id_, rs_frame_id_;
  int analog_hight_noise_th_, analog_max_th_, analog_min_th_, analog_error_value_;
  double sub_tolerance_;
  bool use_observation_source_;

  inline bool checkInvalidValue(int16_t& analog_value)
  {
    return (analog_value < analog_max_th_ && analog_value > analog_min_th_) ? true : false;
  }

  inline auto convertAnalogToMeter(int16_t& analog_value) { return analog_value * 0.001; }

  inline auto convertAnalogDistanceSensorToLaserscan(int16_t analog_value, std::string& frame_id)
  {
    latest_sub_time_ = ros::Time::now();

    sensor_msgs::LaserScan out_scan;
    if (checkInvalidValue(analog_value))
    {
      for (auto i = 0; i < 15; i++) out_scan.ranges.push_back(convertAnalogToMeter(analog_value));
    }
    else
      for (auto i = 0; i < 15; i++) out_scan.ranges.push_back(analog_error_value_);

    out_scan.header.frame_id = frame_id;
    out_scan.header.stamp.sec = ros::Time::now().toSec() - initial_time_.toSec();
    out_scan.header.stamp.nsec = ros::Time::now().nsec - initial_time_.nsec;
    out_scan.scan_time = 0.1;
    out_scan.time_increment = 0.00666;
    out_scan.angle_min = -0.1309;
    out_scan.angle_max = 0.1309;
    out_scan.range_min = 0.1;
    out_scan.range_max = 4.0;
    out_scan.angle_increment = 0.0174533;
    return out_scan;
  }

  inline auto publishScan(sensor_msgs::LaserScan ls_scan, sensor_msgs::LaserScan lf_scan,
                          sensor_msgs::LaserScan rf_scan, sensor_msgs::LaserScan rs_scan)
  {
    ls_pub_.publish(ls_scan);
    lf_pub_.publish(lf_scan);
    rf_pub_.publish(rf_scan);
    rs_pub_.publish(rs_scan);
  }

  void initPub()
  {
    ls_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("/ls_scan", 1);
    lf_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("/lf_scan", 1);
    rf_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("/rf_scan", 1);
    rs_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("/rs_scan", 1);
  }

  void setParam()
  {
    getPrivateNodeHandle().param("left_side_usensor_frame_id", ls_frame_id_,
                                 std::string("left_side_usensor_link"));
    getPrivateNodeHandle().param("left_front_usensor_frame_id", lf_frame_id_,
                                 std::string("left_front_usensor_link"));
    getPrivateNodeHandle().param("right_front_usensor_frame_id", rf_frame_id_,
                                 std::string("right_front_usensor_link"));
    getPrivateNodeHandle().param("right_side_usensor_frame_id", rs_frame_id_,
                                 std::string("right_side_usensor_link"));
    getPrivateNodeHandle().param("usensor_max_threshold", analog_max_th_, 500);
    getPrivateNodeHandle().param("usensor_min_threshold", analog_min_th_, 100);
    getPrivateNodeHandle().param("usensor_error_value", analog_error_value_, 5000);
    getPrivateNodeHandle().param("use_observation_source", use_observation_source_, false);
    getPrivateNodeHandle().param("usensor_topic_receive_tolerance", sub_tolerance_, 1.0);
  }

  void initTimerCb()
  {
    timer_ = getNodeHandle().createTimer(ros::Duration(0.1), [&](auto&) {
      getPrivateNodeHandle().getParam("/move_base/local_costmap/obstacles_layer/raytrace_range",
                                      analog_error_value_);
      if ((ros::Time::now().toSec() - latest_sub_time_.toSec()) > sub_tolerance_)
        ROS_WARN("Not receiving /lightsensors within %f seconds. It actually takes %f seconds.",
                 sub_tolerance_, (ros::Time::now().toSec() - latest_sub_time_.toSec()));
    });
  }

  virtual void onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    ROS_INFO("AnalogDistanceSensorToLaserscan nodelet start........");

    initial_time_ = ros::Time::now();
    latest_sub_time_ = ros::Time::now();

    initPub();
    setParam();
    if (use_observation_source_) initTimerCb();

    light_sensor_subscriber_ = getNodeHandle().subscribe<raspicat::LightSensorValues>(
        "/lightsensors", 10, [&](const auto& msg) {
          raspicat::LightSensorValues in_scan;
          in_scan = *msg;

          publishScan(convertAnalogDistanceSensorToLaserscan(in_scan.left_side, ls_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(in_scan.left_forward, lf_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(in_scan.right_forward, rf_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(in_scan.right_side, rs_frame_id_));
        });
  }
};

}  // namespace analog_distance_sensor_to_laserscan

#endif  // ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H