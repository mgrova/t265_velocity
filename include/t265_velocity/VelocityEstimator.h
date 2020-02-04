#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Eigen>

#include <iostream>
#include <chrono>

#ifndef VELOCITY_ESTIMATOR_H_
#define VELOCITY_ESTIMATOR_H_

class VelocityEstimator{

public:
    VelocityEstimator(){};
    bool init();
    bool setTransformationMatrix(Eigen::Matrix4f _T);
    Eigen::Vector3f currentPosition();
    Eigen::Vector3f previousPosition();
    ros::Publisher publisherVelocity();
    
    Eigen::Vector3f estimateVelocity(Eigen::Vector3f _prevPosition , Eigen::Vector3f _currPosition, float _incT);
    
    void publishROS(ros::Publisher _pub, Eigen::Vector3f _data);

private:
    bool transformReferenceFrame(Eigen::Vector3f _cameraPosition, Eigen::Vector3f &_uavPosition);
    void callbackPose(const nav_msgs::Odometry::ConstPtr& _msg);
private:
    Eigen::Vector3f currPosition_ , prevPosition_;
    Eigen::Matrix4f uavTc_;
    
    ros::NodeHandle nh_;
    ros::Subscriber subPose_;
    ros::Publisher pubVel_;


};

#endif  