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
  ros::Timer timer_;

  boost::mutex connect_mutex_;

  // ROS Parameters
  std::string ls_frame_id_, lf_frame_id_, rf_frame_id_, rs_frame_id_;
  double tolerance_;
  double scan_time_, range_min_, range_max_;

  raspicat::LightSensorValues old_msg_;

  inline auto saveAnalogValue(raspicat::LightSensorValues& new_msg)
  {
    if (checkInvalidValue(new_msg.left_side)) old_msg_.left_side = new_msg.left_side;
    if (checkInvalidValue(new_msg.left_forward)) old_msg_.left_forward = new_msg.left_forward;
    if (checkInvalidValue(new_msg.right_forward)) old_msg_.right_forward = new_msg.right_forward;
    if (checkInvalidValue(new_msg.right_side)) old_msg_.right_side = new_msg.right_side;
  }

  inline auto removalHighNoise(raspicat::LightSensorValues& new_msg) { new_msg = old_msg_; }

  inline bool checkInvalidValue(int16_t& analog_value)
  {
    return (analog_value >= 4000) ? false : true;
  }

  inline auto convertAnalogToMeter(int16_t& analog_value) { return analog_value * 0.001; }

  inline auto convertAnalogDistanceSensorToLaserscan(double analog_value, std::string& frame_id)
  {
    sensor_msgs::LaserScan scan_msg;
    scan_msg.ranges.push_back(analog_value);
    // scan_msg.header.frame_id = frame_id;
    return scan_msg;
  }

  inline auto publishScan(sensor_msgs::LaserScan ls_scan, sensor_msgs::LaserScan lf_scan,
                          sensor_msgs::LaserScan rs_scan, sensor_msgs::LaserScan rf_scan)
  {
    ls_pub_.publish(ls_scan);
    lf_pub_.publish(lf_scan);
    rf_pub_.publish(rf_scan);
    rs_pub_.publish(rs_scan);
  }

  inline auto Initpub()
  {
    ls_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("ls_scan", 1);
    lf_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("lf_scan", 1);
    rf_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("rf_scan", 1);
    rs_pub_ = getNodeHandle().advertise<sensor_msgs::LaserScan>("rs_scan", 1);
  }

  virtual void onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    ROS_INFO("AnalogDistanceSensorToLaserscan nodelet start........");
    getPrivateNodeHandle();

    Initpub();

    light_sensor_subscriber_ = getNodeHandle().subscribe<raspicat::LightSensorValues>(
        "lightsensors", 10, [&](const auto& msg) {
          raspicat::LightSensorValues new_msg;
          new_msg = *msg;

          saveAnalogValue(new_msg);
          removalHighNoise(new_msg);

          publishScan(convertAnalogDistanceSensorToLaserscan(
                          convertAnalogToMeter(new_msg.left_side), ls_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(
                          convertAnalogToMeter(new_msg.left_forward), lf_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(
                          convertAnalogToMeter(new_msg.right_forward), rf_frame_id_),
                      convertAnalogDistanceSensorToLaserscan(
                          convertAnalogToMeter(new_msg.right_side), rs_frame_id_));
        });
  }
};

}  // namespace analog_distance_sensor_to_laserscan

#endif  // ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H