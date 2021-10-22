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

namespace analog_distance_sensor_to_laserscan
{
class AnalogDistanceSensorToLaserscan : public nodelet::Nodelet
{
 public:
  // AnalogDistanceSensorToLaserscan();

 private:
  virtual void onInit()
  {
    ROS_INFO("test");
    std::cout << "test"
              << "\n";
    std::cout << "test" << std::endl;
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_;
  boost::mutex connect_mutex_;

  message_filters::Subscriber<sensor_msgs::LaserScan> sub_;

  // ROS Parameters
  unsigned int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
  double scan_time_, range_min_, range_max_;
  bool use_inf_;
  double inf_epsilon_;
};

}  // namespace analog_distance_sensor_to_laserscan

#endif  // ANALOG_DISTANCE_SENSOR_TO_LASERSCAN_H