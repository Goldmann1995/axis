#include <iostream>
#include "andreffextendedaxxbsolver.h"
#include "conventionalaxxbsvdsolver.h"
#include "extendedaxxbelilambdasvdsolver.h"
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_srvs/Trigger.h>
double axis_roll_offset_deg = 0.0;
std::mutex mutex_axis;
std::mutex mutex_odo;
#define PI_M (3.14159265358)
Poses A ,B;
bool first_axis = true,first_odo = true;
Pose axis_before =  Pose::Identity(4,4);//relative pose
Pose axis_after  =  Pose::Identity(4,4);//relative pose
Pose odo_before =  Pose::Identity(4,4);//relative pose
Pose odo_after  =  Pose::Identity(4,4);//relative pose
void axis_cbk(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lg(mutex_axis);
    double rotator = (msg->fluid_pressure - axis_roll_offset_deg)*PI_M/180.0;
    Eigen::AngleAxis<double> tmp(rotator,Eigen::Vector3d(1,0,0));
    

    if (first_axis)
    {
        axis_before.topLeftCorner(3,3) = tmp.matrix();
        first_axis =false;
    }
    else 
    {
        axis_after.topLeftCorner(3,3)  = tmp.matrix();
        Pose axis = axis_before * axis_after.inverse();
        axis_before = axis_after;
        A.emplace_back(axis);
    }
    std::cout <<"A.size() =" << A.size()<< "\n";
   
}

void imu_cbk(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lg(mutex_odo);

    Eigen::Vector3d T(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    Eigen::Matrix3d rot = q.matrix();

    if (first_odo)
    {
        odo_before.topLeftCorner(3,3) = rot;
        odo_before.topRightCorner(3,1) = T;
        first_odo =false;
    }
    else 
    {
        odo_after.topLeftCorner(3,3)  = rot;
        odo_after.topRightCorner(3,1) = T;
        Pose odo = odo_before * odo_after.inverse();
        odo_before = odo_after;
        B.emplace_back(odo);
    }
    std::cout <<"B.size() =" << B.size()<< "\n";

}

bool commandCallback(std_srvs::Trigger::Request  &req,
         			std_srvs::Trigger::Response &res)
{

    // ExtendedAXXBEliLambdaSVDSolver ex_slover(A,B);
    // Pose X_1 =  ex_slover.SolveX();
    AndreffExtendedAXXBSolver and_slover(A,B);
    Pose X_2 =  and_slover.SolveX();
    // ConventionalAXXBSVDSolver Con_slover(A,B);
    // Pose X_3 =  Con_slover.SolveX();
    // std::cout <<"X_1 =" << X_1<< "\n";
    // std::cout <<"X_2 =" << X_2<< "\n";
    // std::cout <<"X_3 =" << X_3<< "\n";

	// 设置反馈数据
	res.success = true;
	res.message = "Rechnung!";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sloveaxxb");

    ros::NodeHandle nh("~");


    ros::Subscriber sub_imu;
    ros::Subscriber sub_axis;
    std::string imu_topic = "/OdometryPredicted",axis_topic = "/motor_io_node/motor_position";
    sub_imu = nh.subscribe(imu_topic, 100000, imu_cbk, ros::TransportHints().tcpNoDelay());
    sub_axis = nh.subscribe(axis_topic, 10000,  axis_cbk, ros::TransportHints().tcpNoDelay());
    ros::ServiceServer command_service = nh.advertiseService("/cal", commandCallback);
    
    ros::spin();

}


/*

svd_r.matrixV()=  0.994809 -0.0289857   0.097549
                0.0288473    0.99958 0.00282871
                -0.09759          0   0.995227
angle [y,p,r]=  0.0289897   0.0977456 4.33681e-19

svd_r.matrixV()=  0.994962  0.0966006 -0.0267994
                0.026674 0.00258977   0.999641
                -0.0966353    0.99532          0
angle [y,p,r]=0.0268026  0.0967863   -1.5708

svd_r.matrixV()=  0.994739 -0.0264183    0.09898
                0.0262885   0.999651 0.00261579
                -0.0990145          0   0.995086
angle [y,p,r]=   0.0264214     0.099177 -4.33681e-19 ///306


[  0.9947387, -0.0264183,  0.0989799;
   0.0262885,  0.9996510,  0.0026158;
  -0.0990145,  0.0000000,  0.9950860 ]

*/