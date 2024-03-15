
#ifndef gnss
#define gnss

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class pointcloudinterface : public rclcpp::Node
{
public:
   explicit pointcloudinterface(const rclcpp::NodeOptions & node_options);
   virtual ~pointcloudinterface();
  
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_raw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_top;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_con;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr carla_cloud_;
  std::string tf_output;
  void processScan(const sensor_msgs::msg::PointCloud2::SharedPtr scanMsg);
};

#endif 