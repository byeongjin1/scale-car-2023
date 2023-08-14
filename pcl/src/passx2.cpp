#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
ros::Publisher pub;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
pcl::PointCloud<pcl::PointXYZ> inputCloud;
pcl::PointCloud<pcl::PointXYZ> filteredCloud;
pcl::fromROSMsg(*msg, inputCloud);
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (inputCloud.makeShared());
pass.setFilterFieldName ("x");
pass.setFilterLimits (-1.5,0);
pass.setFilterLimitsNegative (true);
pass.filter (filteredCloud);
sensor_msgs::PointCloud2 output;
pcl::toROSMsg(filteredCloud, output);
output.header.frame_id = "/map";
pub.publish(output);
}
int main(int argc, char** argv)
{
ros::init(argc, argv, "passthroughx2");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

pub = nh.advertise<sensor_msgs::PointCloud2>("/passx2", 1);
ros::spin();
return 0;
}
