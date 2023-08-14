#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
ros::Publisher pub;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
pcl::PointCloud<pcl::PointXYZ> inputCloud;
pcl::PointCloud<pcl::PointXYZ> filteredCloud,filteredCloud1,filteredCloud2;
// pcl::PointCloud<pcl::PointXYZ> filteredCloud1,filteredCloud2;
pcl::fromROSMsg(*msg, inputCloud);
pcl::PassThrough<pcl::PointXYZ> pass;

// 범위 지정 1
pass.setInputCloud (inputCloud.makeShared());
pass.setFilterFieldName ("x");
pass.setFilterLimits (-1.0,0.3);

pass.setFilterLimitsNegative (false);
pass.filter (filteredCloud);

// 범위 지정 2
pass.setInputCloud (filteredCloud.makeShared());
pass.setFilterFieldName ("y"); // set Axis(x)
pass.setFilterLimits (0.15, 1.5); // x : -1.0 ~ 1.0
pass.setFilterLimitsNegative (false); // x : -inf ~ -1.0 && 1.0 ~ inf
pass.filter (filteredCloud1);

// 범위 지정 3

pass.setInputCloud (filteredCloud.makeShared());
pass.setFilterFieldName ("y"); // set Axis(x)
pass.setFilterLimits (-1.5, -0.15); // x : -1.0 ~ 1.0
pass.setFilterLimitsNegative (false); // x : -inf ~ -s1.0 && 1.0 ~ inf
pass.filter (filteredCloud2);

filteredCloud= filteredCloud1 + filteredCloud2;
sensor_msgs::PointCloud2 output;
pcl::toROSMsg(filteredCloud, output);
output.header.frame_id = "/map";
//output.header.style = "Points";
pub.publish(output);

}
int main(int argc, char** argv)
{
ros::init(argc, argv, "passPCx");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

pub = nh.advertise<sensor_msgs::PointCloud2>("/passPC1", 1);
ros::spin();
return 0;
}
