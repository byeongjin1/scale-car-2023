#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
ros::Publisher pub;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud_x;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud_y;

    pcl::fromROSMsg(*msg, inputCloud);

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pcl::PassThrough<pcl::PointXYZ> pass_y;


    // X 기준 필터링 
    pass_x.setInputCloud (inputCloud.makeShared());
    pass_x.setFilterFieldName ("x");
    // set Axis(x)
    pass_x.setFilterLimits (-2.1, 1.0);
    // x : -1.0 ~ 1.0
    pass_x.setFilterLimitsNegative (false); //설정영역 또는 이외의 부분 필터링
    pass_x.filter (filteredCloud_x);


    // Y 기준 필터링
    pass_y.setInputCloud (filteredCloud_x.makeShared());
    pass_y.setFilterFieldName ("y");
    // set Axis(x)
    pass_y.setFilterLimits (-1.2, 1.2);
    // y : -1.0 ~ 1.0
    pass_y.setFilterLimitsNegative (false); //설정영역 또는 이외의 부분 필터링
    pass_y.filter (filteredCloud_y);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filteredCloud_y, output);
    output.header.frame_id = "map";
    pub.publish(output);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "passthrough");
    ros::NodeHandle nh;
    // << Subscribe Topic >>
    // topic name : /voxelPC
    // topic type : sensor_msgs::PointCloud2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

    // << Publish Topic >>
    // topic name : /passPC
    // topic type : sensor_msgs::PointCloud2
    pub = nh.advertise<sensor_msgs::PointCloud2>("/passthrough", 1);
    ros::spin();
    return 0;
}
