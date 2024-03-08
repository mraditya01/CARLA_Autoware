
#ifndef MODEL_LOG_NODE_HPP_
#define MODEL_LOG_NODE_HPP_

#include <string>
#include <vector>
#include <ostream>
#include <algorithm>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

class PointCloudInterface : public rclcpp::Node
{
public:
   explicit PointCloudInterface(const rclcpp::NodeOptions & node_options);
   virtual ~PointCloudInterface();
  
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_raw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_top;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_con;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr carla_cloud_;
  std::string tf_output_frame_;
  void processScan(const sensor_msgs::msg::PointCloud2::SharedPtr scanMsg);
  void setupTF();
};

#endif 