#include "gnss_interface/gnss_interface_node.hpp"
#include <proj.h>


namespace interface {
	namespace gnss {

using namespace std;


utils::utils() {
}

utils::~utils() {
}


void utils::convertLLAToXYZ(const std::string& projectionString, const WayPoint& origin, 
                            const double& latitude, const double& longitude, 
                            const double& altitude, double& xOut, double& yOut, double& zOut)
{
    if (projectionString.size() < 8)
        return;

    PJ_CONTEXT *projContext = proj_context_create();
    PJ *projection = proj_create_crs_to_crs(projContext, "EPSG:4326", projectionString.c_str(), NULL);

    if (projection == nullptr)
        return;

    PJ_COORD gpsDegrees = proj_coord(latitude, longitude, altitude, 0);
    PJ_COORD cartesianOut = proj_trans(projection, PJ_FWD, gpsDegrees);
    
    xOut = cartesianOut.enu.e + origin.pos.x;
    yOut = cartesianOut.enu.n + origin.pos.y;
    zOut = cartesianOut.enu.u + origin.pos.z;

    proj_destroy(projection);
    proj_context_destroy(projContext); 
}
} 
} 


void GnssInterface::processGnssData(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
    interface::gnss::WayPoint originalPoint, tfPoint;

    interface::gnss::utils::convertLLAToXYZ(
        "+proj=tmerc +lat_0=0 +lon_0=0 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        originalPoint, message->latitude, message->longitude, message->altitude, 
        tfPoint.pos.x, tfPoint.pos.y, tfPoint.pos.z);

    pose_cov.header = pose_.header;
    pose_cov.pose.pose = pose_.pose;

    pose_.header = message->header;
    pose_.pose.position.x = tfPoint.pos.x;
    pose_.pose.position.y = tfPoint.pos.y;
    pose_.pose.position.z = tfPoint.pos.z;


    pup_pose->publish(pose_);
    pup_pose_cov->publish(pose_cov);
}

GnssInterface::~GnssInterface(){
}

GnssInterface::GnssInterface(const rclcpp::NodeOptions & node_options)
: Node("gnss_interface_node", node_options), tf_output("base_link")
{
	sub_gnss_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		    "carla_nav_sat_fix", 1,
		    std::bind(&GnssInterface::processGnssData, this, std::placeholders::_1));

	pup_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/sensing/gnss/pose", 1);
	pup_pose_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/sensing/gnss/pose_with_covariance", 1);	  
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GnssInterface)
