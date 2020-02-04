

#include <ros/ros.h>
#include <t265_velocity/VelocityEstimator.h>

int main(int _argc , char **_argv){

    ros::init(_argc, _argv, "T265_VELOCITY");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    VelocityEstimator estimator;
    if (!estimator.init())
        return 0;

    ros::Rate r(10); // set 100 Hz 

    auto t0 = std::chrono::system_clock::now();
    while(ros::ok()){

        auto curr = estimator.currentPosition();
        auto prev = estimator.previousPosition();

        auto t1 = std::chrono::system_clock::now();
        float incT = float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
        
        Eigen::Vector3f velocity = estimator.estimateVelocity(prev,curr,incT);
        
        std::cout << "Current position: " << curr[0] << " " << curr[1] << " " << curr[2] << std::endl; 
        // std::cout << "Previous position: " << prev[0] << " " << prev[1] << " " << prev[2] << std::endl; 
        // std::cout << "Time increment: " << incT << std::endl;
        std::cout << "Velocity estimated: " << velocity[0] << " " << velocity[1] << " " << velocity[2] << std::endl;

        estimator.publishROS(estimator.publisherVelocity() , velocity);
        
        t0 = t1;
        r.sleep();
    }



    return 0;
}