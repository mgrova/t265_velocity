#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Eigen>

#include <iostream>


Eigen::Vector3f computeVelocity(Eigen::Vector3f _position, float _incT){
    Eigen::Vector3f velocity;

    // Velocity interpolation


    return velocity;
}

int main(int _argc , char **_argv){

    ros::init(_argc, _argv, "T265_VELOCITY");

    // Extrinsics camera parameters
    Eigen::Vector3f c_t_uav = {1,2,3};
    Eigen::Matrix3f c_R_uav;

    Eigen::Matrix4f uav_T_c;
    uav_T_c.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    uav_T_c.block<3,3>(0,0) = c_R_uav;
    uav_T_c.block<3,1>(0,3) = c_t_uav;

    ros::NodeHandle nh;
    ros::Publisher pubVel = nh.advertise<geometry_msgs::TwistStamped>("/velocity_estimated", 1);
    Eigen::Vector3f currPosition;
    ros::Subscriber subPose = nh.subscribe<geometry_msgs::PoseStamped>("t265/odom/pose",   1, [&](const geometry_msgs::PoseStamped::ConstPtr& _msg){
       position[0] = _msg->pose.position.x; 
       position[1] = _msg->pose.position.y; 
       position[2] = _msg->pose.position.z; 

       // Apply transformation matrix to UAV reference frame

    });

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate r(100); // set 100 Hz 
    
    Eigen::Vector3f prevPosition;

    auto t0 = std::chrono::system_clock::now();
    while(ros::ok()){

        auto t1 = std::chrono::system_clock::now();
        float incT = float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
        Eigen::Vector3f vel = 
        // pubVel.publish();
        ros::spinOnce();
        prevPosition = currPosition;
        t0 = t1;
        r.sleep();
    }



    return 0;
}