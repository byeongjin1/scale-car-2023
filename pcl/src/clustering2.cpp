#include <ros/ros.h> 
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <vector>


ros::Publisher pub1; 
// ros::Publisher pub2;


void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{ 
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::fromROSMsg(*msg, inputCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>); 

    kdtree->setInputCloud (inputCloud.makeShared());

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; 


    ec.setClusterTolerance (0.1);            // set distance threshold = 1.5m 안에 있으면 같은 클러스터 분류로 하겠다.
    ec.setMinClusterSize (5);               // set Minimum Cluster Size  
    ec.setMaxClusterSize (25);            // set Maximum Cluster Size 
    ec.setSearchMethod (kdtree); 
    ec.setInputCloud (inputCloud.makeShared()); 
    ec.extract (clusterIndices);

    int clusterN = 1; // 클러스터가 1개일거다라는 것을 int변수로 선언해주면 된다. 
    pcl::PointCloud<pcl::PointXYZ> aCloud;
    std::vector<pcl::PointIndices>::const_iterator it;

    for (it = clusterIndices.begin (); it != clusterIndices.end (); ++it) 

    {
        pcl::PointCloud<pcl::PointXYZ> filteredCloud;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) 
            filteredCloud.push_back (inputCloud[*pit]);
        aCloud += filteredCloud;



        
        // clusterN == 1 ?  pub1.publish(output) : pub2.publish(output); 
        // clusterN++;
    } 
    sensor_msgs::PointCloud2 output; 
    pcl::toROSMsg(aCloud, output); 
    output.header.frame_id = "/map";
    pub1.publish(output);
}


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "clustering2");
    ros::NodeHandle nh;

    // << Subscribe Topic >> 
    // topic name : /passPC 
    // topic type : sensor_msgs::PointCloud2 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/passy2", 1, callbackFcn);


    // << Publish Topic >> 
    // topic name : /cluster1PC 
    // topic type : sensor_msgs::PointCloud2 
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/clusterPC2", 1);

    // << Publish Topic >> 
    // topic name : /cluster2PC 
    // topic type : sensor_msgs::PointCloud2 
    // pub2 = nh.advertise<sensor_msgs::PointCloud2>("/clusterPC2", 1); 

    ros::spin();
    return 0;
}
