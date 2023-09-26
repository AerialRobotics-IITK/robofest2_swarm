#include "color_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
namespace color
{
    ColorDetector::ColorDetector(ros::NodeHandle nh, std::string image_topic, std::string image_pub_topic)
    {
        nh_ = nh;
        image_sub_ = nh_.subscribe(image_topic, 1, &ColorDetector::imageCallback, this);
        image_pub_ = nh_.advertise<geometry_msgs::Point>(image_pub_topic, 1);
        pub_img = nh_.advertise<sensor_msgs::Image>("/color_detector/contored_image", 1);
    }

    void ColorDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        image = cv_ptr->image;
        cvtColor(image, HSVimage, cv::COLOR_BGR2HSV);
        red_only();
    }
    int ColorDetector::largest_contour_i()
    {
        int largest_area = 0;
        int largest_contour_index = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double a = contourArea(contours[i], false);
            if (a > largest_area)
            {
                largest_area = a;
                largest_contour_index = i;
            }
        }
        return largest_contour_index;
    }
    void ColorDetector::red_only()
    {
        cv::cvtColor(image, HSVimage, cv::COLOR_BGR2HSV);
        cv::inRange(HSVimage, cv::Scalar(0, 50, 50), cv::Scalar(15, 255, 255), detect_screen_red);
        cv::erode(detect_screen_red, detect_screen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)));
        cv::findContours(detect_screen_red, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        image_copy = image.clone();
        cv::drawContours(image_copy, contours,largest_contour_i(), cv::Scalar(0, 255, 0), 2);
        cv::threshold(detect_screen_red, thr, 100, 255, cv::THRESH_BINARY);

        int largest_contour = largest_contour_i();
        ROS_INFO("%ld", contours.size());
        if (largest_contour == 0)
        {
            moments = cv::moments(thr, true);
        }
        else
        {
            moments = cv::moments(contours[largest_contour]);
        }
        cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
        geometry_msgs::Point center_point;
        // area of the largest contour
        if (contours.size() == 0)
        {
            center_point.x = -1;
            center_point.y = -1;
            center_point.z = -1;
            image_pub_.publish(center_point);
            return;
        }

        center_point.x = center.x - (image.cols)/2;
        center_point.y = center.y - (image.rows)/2;
        center_point.z = 1;
        // cv::Point text_position;
        // text_position.x = center_point.x;
        // text_position.y = center_point.y;
        // int font_size = 1;//Declaring the font size//
        // cv::Scalar font_Color(255, 255, 255);
        // std::string text = "Hello";
        // int font_weight = 2;
        // cv::putText(image_copy, text, text_position, cv::FONT_HERSHEY_COMPLEX, font_size, (255, 255, 255), font_weight);
        cv::Point text_position((float)center_point.x + (image.cols)/2, (float)center_point.y+(image.rows)/2);//Declaring the text position//
        int font_size = 1;//Declaring the font size//

        // std::string str_x = std::to_string(center_point.x);
        // std::string str_y = std::to_string(center_point.y);
        // std::string str_z = std::to_string(center_point.z);
        ROS_INFO("%d, %d. %d",center_point.x,center_point.y, center_point.z);

        // std::string str_final = "(" + str_x + "," + str_y + "," + str_z + ")";

        // cv::Scalar font_Color(255, 255, 255);//Declaring the color of the font//
        // int font_weight = 1;//Declaring the font weight//
        // cv::putText(image_copy, "()", text_position,cv::FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);//Putting the text in the matrix//
   
        std_msgs::Header header = cv_ptr->header;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_copy);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        pub_img.publish(img_msg);


        image_pub_.publish(center_point);
    }
} // namespace color