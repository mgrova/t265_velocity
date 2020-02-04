#include <t265_velocity/VelocityEstimator.h>


bool VelocityEstimator::init(){
        // Extrinsics camera parameters
    Eigen::Vector3f c_t_uav;
    Eigen::Matrix3f c_R_uav;

    Eigen::Matrix4f uav_T_c; //666    
    uav_T_c.setIdentity();  
    uav_T_c.block<3,3>(0,0) = c_R_uav;
    uav_T_c.block<3,1>(0,3) = c_t_uav;

    setTransformationMatrix(uav_T_c);
    
    pubVel_ = nh_.advertise<geometry_msgs::TwistStamped>("/velocity_estimated", 1);
    subPose_ = nh_.subscribe("/camera/odom/sample", 1, &VelocityEstimator::callbackPose, this);        
    
    return true;
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

ros::Publisher VelocityEstimator::publisherVelocity(){
    return pubVel_;
}


void VelocityEstimator::publishROS(ros::Publisher _pub, Eigen::Vector3f _data){
    geometry_msgs::TwistStamped estVelocity;
    estVelocity.header.frame_id = "camera_link";  // 666
    estVelocity.header.stamp=ros::Time::now();
    estVelocity.twist.linear.x = _data[0];
    estVelocity.twist.linear.y = _data[1];
    estVelocity.twist.linear.z = _data[2];

    _pub.publish(estVelocity);

    return;

}

bool VelocityEstimator::transformReferenceFrame(Eigen::Vector3f _cameraPosition, Eigen::Vector3f &_uavPosition){
    // Transform position from camera frame to UAV frame


}

void VelocityEstimator::callbackPose(const nav_msgs::Odometry::ConstPtr& _msg){

    if (!currPosition_.isZero())
        prevPosition_ = currPosition_;

    Eigen::Vector3f position;
    position[0] =   _msg->pose.pose.position.x; 
    position[1] = - _msg->pose.pose.position.y; 
    position[2] =   _msg->pose.pose.position.z; 

    //transformReferenceFrame(position,currPosition_);
    currPosition_ = position; // 666 must apply tranformation to UAV reference frame

    return;
}

Eigen::Vector3f VelocityEstimator::estimateVelocity(Eigen::Vector3f _prevPosition , Eigen::Vector3f _currPosition, float _incT){
    Eigen::Vector3f vel;

    vel[0] = (_currPosition[0] - _prevPosition[0]) / _incT;
    vel[1] = (_currPosition[1] - _prevPosition[1]) / _incT;
    vel[2] = (_currPosition[2] - _prevPosition[2]) / _incT;

    return vel;
}