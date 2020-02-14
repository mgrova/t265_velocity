//---------------------------------------------------------------------------------------------------------------------
//  VELOCITY ESTIMATOR REALSENSE T265
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

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
    
    bool estimateVelocity(Eigen::Vector3f _prevPosition , Eigen::Vector3f _currPosition, float _incT);
    
    /// Publish position and velocity on UAV coordinate system
    void publishROS();

private:
    bool transformReferenceFrame(Eigen::Vector3f _cameraPosition, Eigen::Vector3f &_uavPosition);
    void callbackOdom(const nav_msgs::Odometry::ConstPtr& _msg);
private:
    /// Position and velocity referenced to UAV coordinate system
    Eigen::Vector3f currPosition_ , prevPosition_ , velocity_;
    // Transformation between camera and UAV coordinate system
    Eigen::Matrix4f uavTc_;
    
    ros::NodeHandle nh_;
    ros::Subscriber subPose_;
    ros::Publisher pubVelCamera_,pubVelEstimat_, pubPose_;


};

#endif  
