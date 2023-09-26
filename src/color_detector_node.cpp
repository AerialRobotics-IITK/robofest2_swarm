#include "color_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_detector_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    // image_transport::Subscriber cam_sub = it.subscribe("/camera/image_raw", 1, imageCallback);
    color::ColorDetector color_detector(nh, "/roscam/cam/image_raw", "/color_detector/color_detected");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}