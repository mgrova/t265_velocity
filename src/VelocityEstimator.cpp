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

#include <t265_velocity/VelocityEstimator.h>


bool VelocityEstimator::init(){
    // Extrinsics camera parameters
    // uav_T_c = [c_R_uav c_t_uav ;
    //             [000]     1    ]
    Eigen::Vector3f c_t_uav(0.08 , 0.0 , -0.18);  //666 units in meters.
    Eigen::Matrix3f c_R_uav =Eigen::Matrix3f::Identity();

    Eigen::Matrix4f uav_T_c; 
    uav_T_c.setIdentity();  
    uav_T_c.block<3,3>(0,0) = c_R_uav;
    uav_T_c.block<3,1>(0,3) = c_t_uav;

    setTransformationMatrix(uav_T_c);
    
	pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/external_estimated/position", 1);
    pubVelEstimat_ = nh_.advertise<geometry_msgs::TwistStamped>("/external_estimated/velocity", 1);
    pubVelCamera_  = nh_.advertise<geometry_msgs::TwistStamped>("/t265/velocity", 1);
    subPose_ = nh_.subscribe("/camera/odom/sample", 1, &VelocityEstimator::callbackOdom, this);        
    
    return true;
}


void VelocityEstimator::callbackOdom(const nav_msgs::Odometry::ConstPtr& _msg){

    if (!currPosition_.isZero())
        prevPosition_ = currPosition_;

    Eigen::Vector3f position;
    position[0] =   _msg->pose.pose.position.x; //  x_cam = x_uav
    position[1] = - _msg->pose.pose.position.y; // -y_cam = y_uav
    position[2] =   _msg->pose.pose.position.z; //  z_cam = z_uav

    geometry_msgs::TwistStamped VelocityCamera;
    VelocityCamera.header.frame_id = "local_NEU"; 
    VelocityCamera.header.stamp = ros::Time::now();
    VelocityCamera.twist.linear.x =   _msg->twist.twist.linear.x;
    VelocityCamera.twist.linear.y = - _msg->twist.twist.linear.y;
    VelocityCamera.twist.linear.z =   _msg->twist.twist.linear.z;

    pubVelCamera_.publish(VelocityCamera);

    transformReferenceFrame(position,currPosition_);
    currPosition_ = position; 
    
    
    return;
}


bool VelocityEstimator::setTransformationMatrix(Eigen::Matrix4f _T){
    uavTc_ = _T;

    return true;
}

Eigen::Vector3f VelocityEstimator::currentPosition(){
    return currPosition_;
}

Eigen::Vector3f VelocityEstimator::previousPosition(){
    return prevPosition_;
}

void VelocityEstimator::publishROS(){
   	geometry_msgs::PoseStamped positionUAV;
  	positionUAV.header.stamp = ros::Time::now();;
   	positionUAV.header.frame_id = "local_NEU";
   	positionUAV.pose.position.x = currPosition_[0];
    positionUAV.pose.position.y = currPosition_[1];
    positionUAV.pose.position.z = currPosition_[2];

	geometry_msgs::TwistStamped estVelocityUAV;
    estVelocityUAV.header.frame_id = "local_NEU"; 
    estVelocityUAV.header.stamp = positionUAV.header.stamp;
    estVelocityUAV.twist.linear.x = velocity_[0];
    estVelocityUAV.twist.linear.y = velocity_[1];
    estVelocityUAV.twist.linear.z = velocity_[2];
	
	pubPose_.publish(positionUAV);
    pubVelEstimat_.publish(estVelocityUAV);

    return;

}

bool VelocityEstimator::transformReferenceFrame(Eigen::Vector3f _cameraPosition, Eigen::Vector3f &_uavPosition){
    Eigen::Vector4f normUAVPosition;
    Eigen::Vector4f normCameraPosition(_cameraPosition[0],_cameraPosition[1],_cameraPosition[2],1);

    // Transform position from camera frame to UAV frame
    normUAVPosition = uavTc_ * normCameraPosition;

    _uavPosition = Eigen::Vector3f(normUAVPosition[0] , normUAVPosition[1] , normUAVPosition[2]);

    return true;
}

bool VelocityEstimator::estimateVelocity(Eigen::Vector3f _prevPosition , Eigen::Vector3f _currPosition, float _incT){
    Eigen::Vector3f vel;

    vel[0] = (_currPosition[0] - _prevPosition[0]) / _incT;
    vel[1] = (_currPosition[1] - _prevPosition[1]) / _incT;
    vel[2] = (_currPosition[2] - _prevPosition[2]) / _incT;

    velocity_ = vel;
    return true;
}
