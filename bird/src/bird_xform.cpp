#include "ros/ros.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

class Bird {
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber it_sub;
    image_transport::Publisher it_pub;
    cv::Mat H;
    cv::Mat intrinsic;
    cv::Mat distortion;

public:
    Bird() : it(n){
        it_sub = it.subscribe("cam_segment", 1, &Bird::got_frame, this);
        it_pub = it.advertise("cam_bird", 1);

        //read in homography matrix
        cv::FileStorage fs("/home/ken/spring12/robotics/ros_workspace/igvc-2012/bird/data/H.xml", cv::FileStorage::READ);
    
        H.create(3, 3, CV_32F);
        fs["H"] >> H;
        fs.release();
        
        fs.open("/home/ken/spring12/robotics/ros_workspace/igvc-2012/bird/data/Intrinsics.xml", cv::FileStorage::READ);
        intrinsic.create(3, 3, CV_32F);
        fs["Intrinsics"] >> intrinsic;
        fs.release();

        
        fs.open("/home/ken/spring12/robotics/ros_workspace/igvc-2012/bird/data/Distortion.xml", cv::FileStorage::READ);
        distortion.create(4, 1, CV_32F);
        fs["Distortion"] >> distortion;
        fs.release();
    }

    ~Bird(){
        H.release();
    }

    void got_frame(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr frame;
        cv_bridge::CvImage out_topic;

        //bridge
        try{
            frame = cv_bridge::toCvCopy(msg, enc::TYPE_8UC1);
        }catch(cv_bridge::Exception &e){
            ROS_ERROR("bird exception: %s", e.what());
            return;
        }

        cv::Mat out(frame->image.rows, frame->image.cols, CV_8UC1);

        //out = frame->image.clone();

        //undistort
        cv::Mat t;
        t  = frame->image.clone();

        //cv::undistort(frame->image, t, intrinsic, distortion);

        cv::warpPerspective(t, out, H, out.size(), CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
        
        out_topic.header = frame->header;
        out_topic.encoding = enc::TYPE_8UC1;
        out_topic.image = out;
        
        it_pub.publish(out_topic.toImageMsg());
        //out.release();
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "bird_xform");
    Bird oop;
    ros::spin();
    return 0;
}
