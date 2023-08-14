#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
ros::Publisher pub;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
pcl::PointCloud<pcl::PointXYZ> inputCloud;
pcl::PointCloud<pcl::PointXYZ> filteredCloud;

pcl::fromROSMsg(*msg, inputCloud);
pcl::VoxelGrid<pcl::PointXYZ> vox;
vox.setInputCloud (inputCloud.makeShared());
vox.setLeafSize (0.85f,0.85f,0.85f);
vox.filter (filteredCloud);
sensor_msgs::PointCloud2 output;
pcl::toROSMsg(filteredCloud, output);
output.header.frame_id = "/map";
pub.publish(output);
}
int main(int argc, char** argv) {
ros::init(argc, argv, "voxel");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel", 1);
ros::spin();
return 0; }
