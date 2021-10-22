#ifndef ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H
#define ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H

#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
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
  std::string ls_frame_id, lf_frame_id, rf_frame_id, rs_frame_id;
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

  inline auto convertAnalogDistanceSensorIntoLaserscan(int16_t& analog_value)
  {
    return analog_value * 0.001;
  }

  virtual void onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    ROS_INFO("AnalogDistanceSensorToLaserscan nodelet start........");
    getPrivateNodeHandle();

    light_sensor_subscriber_ = getNodeHandle().subscribe<raspicat::LightSensorValues>(
        "lightsensors", 10, [&](const auto& msg) {
          raspicat::LightSensorValues new_msg;
          new_msg = *msg;

          saveAnalogValue(new_msg);
          removalHighNoise(new_msg);

          std::cout << convertAnalogToMeter(new_msg.left_side) << ", ";
          std::cout << convertAnalogToMeter(new_msg.left_forward) << ", ";
          std::cout << convertAnalogToMeter(new_msg.right_forward) << ", ";
          std::cout << convertAnalogToMeter(new_msg.right_side) << "\n";
        });
  }
};

}  // namespace analog_distance_sensor_to_laserscan

#endif  // ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H