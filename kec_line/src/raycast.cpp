#include "ros/ros.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "naive/naive.hpp"

namespace enc = sensor_msgs::image_encodings;

class ToLaserScan {
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber it_sub;
    ros::Publisher scan_pub;

public:
    ToLaserScan() : it(n){
        it_sub = it.subscribe("cam_segment", 1, &ToLaserScan::got_frame, this);
        scan_pub = n.advertise<sensor_msgs::LaserScan>("cam_laser", 50);
    }

    ~ToLaserScan(){
    }

    void got_frame(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr frame;
        sensor_msgs::LaserScan scan;

        //bridge
        try{
            frame = cv_bridge::toCvCopy(msg, enc::TYPE_8UC1);
        }catch(cv_bridge::Exception &e){
            ROS_ERROR("kec_line raycast exception: %s", e.what());
            //std::cout << "kec_line exception: " << e.what() << std::endl;
            return;
        }

        raycast(&frame->image, scan);

        //publish output topic
        scan_pub.publish(scan);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "raycast");
    ToLaserScan oop;
    ros::spin();
    return 0;
}
