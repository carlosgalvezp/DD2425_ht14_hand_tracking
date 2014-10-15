#include <iostream>
// ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// PCL

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE   10

#define DISTANCE_THRESHOLD 100

class Object_Detection
{
    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Object_Detection(const ros::NodeHandle& n);
    void run(int argc, char *argv[]);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;


    void RGBD_Callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);
};

// =============================================================================
// =============================================================================

Object_Detection::Object_Detection(const ros::NodeHandle &n)
    : n_(n), rgb_transport_(n), depth_transport_(n)
{
    // ** Publishers
//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // ** Subscribers
    rgb_sub_.subscribe(rgb_transport_,
                       "/camera/rgb/image_raw", QUEUE_SIZE);
    depth_sub_.subscribe(depth_transport_,
                       "/camera/depth/image_raw", QUEUE_SIZE);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Object_Detection::RGBD_Callback, this, _1, _2));

}

void Object_Detection::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                     const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;
    cv::imshow("RGB", rgb_img);
    cv::imshow("DEPTH", depth_img);
    cv::waitKey(1);

    cv::Mat m2(depth_img);
    m2.convertTo(m2, CV_8U);
    cv::Mat mask;
    cv::threshold(m2, mask, DISTANCE_THRESHOLD, 255, cv::THRESH_BINARY);
    cv::imshow("MASK", mask);
    cv::waitKey(1);
}

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle n;

    // ** Create object detection object
    Object_Detection o(n);

    // ** Spin
    ros::spin();
    return 0;
}
