#ifndef IMAGE_CONVERTER_HPP
#define IMAGE_CONVERTER_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "person_tracker.hpp"
#include "vz_face_recognition/HSE_Object.h"
#include "vz_face_recognition/HSE_Objects.h"

#include <string>
#include <vector>

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    // ros::Subscriber fr_sub;
    ros::Publisher pt_pub_;

public:
    ImageConverter();

    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    // void facerecCb(const  vz_face_recognition::HSE_Objects::ConstPtr &msg);
    PersonTracker person_tracker;
    std::vector< std::vector< int > > rois;
    std::vector< std::string > labels;
};

#endif