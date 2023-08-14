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
pass.setFilterFieldName ("y"); // set Axis(x)
pass.setFilterLimits (-0.8, -0.15); // x : -1.0 ~ 1.0
pass.setFilterLimitsNegative (false); // x : -inf ~ -1.0 && 1.0 ~ inf
pass.filter (filteredCloud);
sensor_msgs::PointCloud2 output;
pcl::toROSMsg(filteredCloud, output);
output.header.frame_id = "/map";
pub.publish(output);
}
int main(int argc, char** argv)
{
ros::init(argc, argv, "passthroughy1");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/passx1", 1000, callbackFcn);

pub = nh.advertise<sensor_msgs::PointCloud2>("/passy1", 1000);
ros::spin();
return 0;
}
