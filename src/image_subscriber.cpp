#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 显示图像
        cv::imshow("Received Image", image);
        cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber");
    ros::NodeHandle nh;
    std::string topic_name = std::string(argv[1])+"/image_raw";

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic_name, 10, imageCallback);
    
    ros::spin();
    return 0;
}