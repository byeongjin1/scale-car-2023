#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <math.h>
#include <time.h>
#include <unistd.h>
using namespace std;
ros::Publisher pub, pub2,pub3;
double error_prev=0;



double get_error(geometry_msgs::Point pR, geometry_msgs::Point pL)
{
    
    double theta = (pR.y+pL.y)/2;
    // cout <<"theta is: " << theta<< '\n';

    return theta;
}



double PID_control(double error, double *error_prev)
{
    float kp=3;  //change kp
    float output = -kp*error;
    // cout <<"prev_error is: " << *error_prev<< '\n';
    *error_prev = error;
    return output;
}



double TH_control(double error, double *error_prev)  // 포인터변수 error_prev
{
    double output;
    if(error > 0.2){
        output = -0.6;
    }
    else if(error>=0.15){
        output = -0.38;
    }
    else if(error>=0.10){
        output = -0.15;
    }

    else if(error <= -0.2){
        output = 0.6;

    }
    else if(error<=-0.15){
       output = 0.48;
    }
    else if(error<=-0.1){
        output = 0.2;
    }
    else{
        output = 0;
    }
    cout <<"control : " << output<< '\n';

    // cout <<"prev_error is: " << *error_prev<< '\n';
    *error_prev = error; //error값을 error_prev주소값에 값으로 저장
    return output;
}

void callback(const sensor_msgs::PointCloud2 msg)
{
    visualization_msgs::Marker line_listL, line_listR;
    geometry_msgs::Point pL, pR ,p1L,p1R;

    line_listL.header.frame_id = "/map";
    line_listL.header.stamp = ros::Time::now();
    line_listL.ns = "points_and_linesL";
    line_listL.action = visualization_msgs::Marker::ADD;
    line_listL.pose.orientation.w = 1.0;
    line_listL.id = 1;
    line_listL.type = visualization_msgs::Marker::LINE_LIST;
    line_listL.scale.x = 0.1;
    line_listL.color.b = 1.0;
    line_listL.color.a = 1.0;

    line_listR.header.frame_id = "/map";
    line_listR.header.stamp = ros::Time::now();
    line_listR.ns = "points_and_linesR";
    line_listR.action = visualization_msgs::Marker::ADD;
    line_listR.pose.orientation.w = 1.0;
    line_listR.id = 2;
    line_listR.type = visualization_msgs::Marker::LINE_LIST;
    line_listR.scale.x = 0.1;
    line_listR.color.r = 1.0;
    line_listR.color.a = 1.0;

    sensor_msgs::PointCloud out_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
    vector<double> axL,ayL,azL, axR, ayR, azR;
    // y = ax + b
    double aL, bL, xbarL=0, ybarL=0, sum1L=0, sum2L=0, aR, bR, xbarR=0, ybarR=0, sum1R=0, sum2R=0;
    int n= out_cloud.points.size(),nR=0,nL=0;
    double error=0;


    for(int i = 0 ; i <= n-1; i++){
        geometry_msgs::Point32 pointL, pointR;


        if(out_cloud.points[i].y<=0.15){
            pointL.x = out_cloud.points[i].x;
            pointL.y = out_cloud.points[i].y;
            axL.push_back(pointL.x);
            ayL.push_back(pointL.y);
            xbarL += pointL.x;
            ybarL += pointL.y;
            nL += 1;

        }
        
        else if(out_cloud.points[i].y>-0.15){
            pointR.x = out_cloud.points[i].x;
            pointR.y = out_cloud.points[i].y;
            axR.push_back(pointR.x);
            ayR.push_back(pointR.y);
            xbarR += pointR.x;
            ybarR += pointR.y;
            nR += 1;
        }

    }

    
    std_msgs::Float64 angle1;
    std_msgs::Float64 vel1;

    if(axL.size() and axR.size()){    
        double xxL2 = *max_element(axL.begin(),axL.end());
        double xxR2 = *max_element(axR.begin(),axR.end());

        xbarL = xbarL/nL;
        ybarL = ybarL/nL;

        xbarR = xbarR/nR;
        ybarR = ybarR/nR;

        for(int i = 0 ; i <= nL-1; i++){
            sum1L += (axL[i]-xbarL)*(ayL[i]-ybarL);
            sum2L += (axL[i]-xbarL)*(axL[i]-xbarL);
            
        }
        for(int i = 0 ; i <= nR-1; i++){
            sum1R += (axR[i]-xbarR)*(ayR[i]-ybarR);
            sum2R += (axR[i]-xbarR)*(axR[i]-xbarR);
        }

        aL = sum1L / sum2L;
        bL = ybarL - (aL*xbarL);


        pL.x = -1.5;
        pL.y = aL*(pL.x)+bL;
        line_listL.points.push_back(pL);
        pL.x = 0;
        pL.y = aL*(pL.x)+bL;
        line_listL.points.push_back(pL);
        pub.publish(line_listL);

        p1L.x = -0.5;  // change -0.5 
        p1L.y = aL*(p1L.x)+bL;
       
    

        aR = sum1R / sum2R;
        bR = ybarR - (aR*xbarR);
        pR.x = -1.50;
        pR.y = aR*(pR.x)+bR;
        line_listR.points.push_back(pR);
        pR.x = 0;
        pR.y = aR*(pR.x)+bR;
        line_listR.points.push_back(pR);
        pub.publish(line_listR);

        p1R.x = -0.5;  //
        p1R.y = aR*(p1R.x)+bR;
       
      

        error = get_error(p1R,p1L);
    }

    else if(axL.size()){
        double xxL2 = *max_element(axL.begin(),axL.end());

        xbarL = xbarL/nL;
        ybarL = ybarL/nL;


        for(int i = 0 ; i <= nL-1; i++){
            sum1L += (axL[i]-xbarL)*(ayL[i]-ybarL);
            sum2L += (axL[i]-xbarL)*(axL[i]-xbarL);
            
        }
        aL = sum1L / sum2L;
        bL = ybarL - (aL*xbarL);


        pL.x = -1.5;
        pL.y = aL*(pL.x)+bL;
        line_listL.points.push_back(pL);
        pL.x = 0;
        pL.y = aL*(pL.x)+bL;
        line_listL.points.push_back(pL);
        pub.publish(line_listL);

        p1L.x = -0.5;  //
        p1L.y = aL*(p1L.x)+bL;
     

       
        error = get_error(p1R,p1L);
    }
    else if(axR.size()){
        double xxR2 = *max_element(axR.begin(),axR.end());

        xbarR = xbarR/nR;
        ybarR = ybarR/nR;


        for(int i = 0 ; i <= nR-1; i++){
            sum1R += (axR[i]-xbarR)*(ayR[i]-ybarR);
            sum2R += (axR[i]-xbarR)*(axR[i]-xbarR);
        }

        aR = sum1R / sum2R;
        bR = ybarR - (aR*xbarR);
        pR.x = -1.5;
        pR.y = aR*(pR.x)+bR;
        line_listR.points.push_back(pR);
        pR.x = 0;
        pR.y = aR*(pR.x)+bR;
        line_listR.points.push_back(pR);
        pub.publish(line_listR);

        p1R.x = -0.5;   
        p1R.y = aR*(p1R.x)+bR;
       

        error = get_error(p1R,p1L);
    }
        float angle = PID_control(error, &error_prev);  
        angle1.data = angle;
        vel1.data = 2.5;

        if (angle <= 0.1){
        vel1.data = 2.8;
        }

        pub3.publish(vel1);
        pub2.publish(angle1);
 
    cout <<"angle is: " << angle << '\n';

    }
    


    // cout <<"(atan)theta is: " << qq << '\n';
    // cout <<"(atan2)theta is: " << qq2 << '\n';
    // cout <<"aR: " << aR << '\n';
    // cout <<"aL: " << aL << '\n';

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laneFitting");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub=n.subscribe("/clusterPC", 100, callback);
  pub=n.advertise<visualization_msgs::Marker>("/input", 10);
  pub2=n.advertise<std_msgs::Float64>("/del", 10);
  pub3=n.advertise<std_msgs::Float64>("/vel", 10);
  ros::spin();
  
  return 0;
}
