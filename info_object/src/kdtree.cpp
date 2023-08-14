#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

ros::Publisher pub;

void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    pcl::fromROSMsg(*msg, inputCloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (inputCloud.makeShared());

    pcl::PointXYZ searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    float radius = 3; // set searching radius size = 3m
    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        for(int i = 0; i < pointIdxRadiusSearch.size (); ++i)
            filteredCloud.points.push_back(inputCloud.points[pointIdxRadiusSearch[i]]);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filteredCloud, output);
    output.header.frame_id = "map";

    pub.publish(output);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "kdtree");
    ros::NodeHandle nh;
    // << Subscribe Topic >>
    // topic name : /vLidarPC
    // topic type : sensor_msgs::PointCloud2
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/vLidarPC", 1, callbackFcn);
    // << Publish Topic >>
    // topic name : /kdtreePC
    // topic type : sensor_msgs::PointCloud2
    pub = nh.advertise<sensor_msgs::PointCloud2>("/kdtreePC", 1);

    ros::spin();

    return 0;
}