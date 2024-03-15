#include "pointcloud_interface/pointcloud_interface_node.hpp"

pointcloudinterface::~pointcloudinterface() {
}

pointcloudinterface::pointcloudinterface(const rclcpp::NodeOptions &node_options)
    : Node("carla_pointcloud_interface_node", node_options), tf_output("base_link") {

    carla_cloud_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "carla_pointcloud", rclcpp::SensorDataQoS(), std::bind(&pointcloudinterface::processScan, this, std::placeholders::_1));

    velodyne_points_raw =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points_raw", rclcpp::SensorDataQoS());

    velodyne_points_top =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/outlier_filtered/pointcloud", rclcpp::SensorDataQoS());

    velodyne_points_con =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS());
}

void pointcloudinterface::processScan(const sensor_msgs::msg::PointCloud2::SharedPtr scanMsg) {
    sensor_msgs::msg::PointCloud2 relay_cloud;
    relay_cloud.header.stamp = scanMsg->header.stamp;

    velodyne_points_top->publish(relay_cloud);
    velodyne_points_con->publish(relay_cloud);
    velodyne_points_raw->publish(relay_cloud);
}



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloudinterface)
