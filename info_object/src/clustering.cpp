#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <info_object/ObjectInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher ClusterPub;
ros::Publisher ObjectInfoPub;
ros::Publisher boundingBoxPub;

info_object::ObjectInfo objectInfoMsg;
using namespace std;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){

    //boundingBox 정의
    visualization_msgs::Marker boundingBox;
    visualization_msgs::MarkerArray boundingBoxArray;
    
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::fromROSMsg(*msg, inputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud (inputCloud.makeShared());

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.08); // 점과 점 사이 간격(클러스터링)
    ec.setMinClusterSize (5); // 최소 클러스터링 점
    ec.setMaxClusterSize (200); // 최대 클러스터링 점
    ec.setSearchMethod (kdtree); // 클러스터링 알고리즘
    ec.setInputCloud (inputCloud.makeShared());
    ec.extract (clusterIndices);


    pcl::PointCloud<pcl::PointXYZI> total_cloud;
    int clusterN = 0;
    std::vector<pcl::PointIndices>::const_iterator it;
    for (it = clusterIndices.begin (); it != clusterIndices.end (); ++it){
        float cluster_counts = clusterIndices.size();
        pcl::PointCloud<pcl::PointXYZI> each_cloud;
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZI tmp;
            tmp.x = inputCloud.points[*pit].x;
            tmp.y = inputCloud.points[*pit].y;
            tmp.z = inputCloud.points[*pit].z;
            tmp.intensity = clusterN;
            each_cloud.push_back (tmp);
            total_cloud.push_back (tmp);
        }

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(each_cloud, minPoint, maxPoint);

        float x_len = abs(maxPoint.x - minPoint.x);   //직육면체 x 모서리 크기
        float y_len = abs(maxPoint.y - minPoint.y);   //직육면체 y 모서리 크기
        float z_len = abs(maxPoint.z - minPoint.z);   //직육면체 z 모서리 크기 
        float volume = x_len * y_len * z_len;         //직육면체 부피 (쓸일이 없다)
        float center_x = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        float center_y = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        float center_z = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표 
        float distance = sqrt(center_x * center_x + center_y * center_y); //장애물 <-> 차량 거리(라이다는 0,0,0 이다.)

        //rviz로 나타내보자!
        boundingBox.header.frame_id = "map";
        boundingBox.header.stamp = ros::Time(); // 식별자. 시간을 넣어줌
        boundingBox.ns = cluster_counts; //ns = namespace
        boundingBox.id = clusterN; 
        boundingBox.type = visualization_msgs::Marker::CYLINDER; //직육면체로 표시
        boundingBox.action = visualization_msgs::Marker::ADD;

        boundingBox.pose.position.x = center_x; 
        boundingBox.pose.position.y = center_y;
        boundingBox.pose.position.z = center_z;

        boundingBox.pose.orientation.x = 0.0; //무시해도 됨
        boundingBox.pose.orientation.y = 0.0;
        boundingBox.pose.orientation.z = 0.0;
        boundingBox.pose.orientation.w = 1.0;

        boundingBox.scale.x = 0.2;
        boundingBox.scale.y = 0.2;
        boundingBox.scale.z = 0.5;

        boundingBox.color.a = 1.0; //직육면체 투명도, a = alpha
        boundingBox.color.r = 0.0; //직육면체 색상 RGB값
        boundingBox.color.g = 0.0;
        boundingBox.color.b = 1.0;
        boundingBox.lifetime = ros::Duration(0.1); //box 지속시간(길면은 bounding box 길이가 계속 머무른다. 짧으면 실시간성이 높아진다)
        boundingBoxArray.markers.emplace_back(boundingBox); // push_back 보다 속도가 빠르다.
        boundingBoxPub.publish(boundingBoxArray);

        
        //내가만든 msg형식에 필요한 정보 담아서 보내기
        objectInfoMsg.lengthX[clusterN] = maxPoint.x - minPoint.x; // 
        objectInfoMsg.lengthY[clusterN] = maxPoint.y - minPoint.y; // 
        objectInfoMsg.lengthZ[clusterN] = maxPoint.z - minPoint.z; // 0
        objectInfoMsg.centerX[clusterN] = (minPoint.x + maxPoint.x)/2; 
        objectInfoMsg.centerY[clusterN] = (minPoint.y + maxPoint.y)/2; 
        objectInfoMsg.centerZ[clusterN] = (minPoint.z + maxPoint.z)/2; //0 


        clusterN++;
    }

    // total clustering 전달
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(total_cloud, output);
    output.header.frame_id = "map";
    ClusterPub.publish(output);

    // 객체정보값 전달
    objectInfoMsg.objectCounts = clusterN;
    ObjectInfoPub.publish(objectInfoMsg);
    cout << "clustering number : " << clusterN << endl;
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/passthrough", 1, callbackFcn);
    ClusterPub = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
    ObjectInfoPub = nh.advertise<info_object::ObjectInfo>("object_info", 1);
    boundingBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/boundingBox", 1);
    ros::spin();
    return 0;
}
