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
#include <t265_velocity/VelocityEstimator.h>

int main(int _argc , char **_argv){

    ros::init(_argc, _argv, "T265_VELOCITY");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    VelocityEstimator estimator;
    if (!estimator.init())
        return 0;

    std::cout << " Publishing position and velocity from T265 in UAV coordinate system \n";
    ros::Rate r(50); // set 100 Hz 
    auto t0 = std::chrono::system_clock::now();
    while(ros::ok()){

        auto curr = estimator.currentPosition();
        auto prev = estimator.previousPosition();

        auto t1 = std::chrono::system_clock::now();
        float incT = float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
        
        if (estimator.estimateVelocity(prev,curr,incT)){
            estimator.publishROS();
        }

        t0 = t1;
        r.sleep();
    }



    return 0;
}