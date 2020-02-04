#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Eigen>

#include <iostream>


std::vector<float> computeVelocity(std::vector<float> _position){
    std::vector<float> velocity;
    // Velocity interpolation


    return velocity;
}

int main(int _argc , char **_argv_){

    ros::init(_argc, _argv, "T265_VELOCITY");

    // Extrinsics camera parameters
    Eigen::Vector3f c_t_uav = {1,2,3};
    Eigen::Matrix3f c_R_uav;

    Eigen::Matrix4f uav_T_c;
    uav_T_c.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    uav_T_c.block<3,3>(0,0) = c_R_uav;
    uav_T_c.block<3,1>(0,3) = c_t_uav;

    ros::NodeHandler nh;
    ros::Publisher pubVel = nh.advertise<geometry_msgs::TwistStamped>("/velocity_estimated", 1);
    Eigen::Vector3f position ;
    ros::Subscriber subPose = nh.subscribe<geometry_msgs::PoseStamped>(GPSTopic,   1, [](const geometry_msgs::PoseStamped::ConstPtr& _msg){
       position[0] = _msg.pose.position.x; 
       position[1] = _msg.pose.position.y; 
       position[2] = _msg.pose.position.z; 

       // Apply transformation matrix to UAV reference frame

    });


    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate r(100);
    while(ros.ok()){


        // pubVel.publish();
        ros::spinOnce();
        r.sleep();
    }



    return 0;
}