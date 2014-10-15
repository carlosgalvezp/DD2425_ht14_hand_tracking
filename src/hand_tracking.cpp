#include <iostream>
// ROS
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// PCL

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE   10

#define DISTANCE_THRESHOLD_MULTIPLIER 1.25

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
    void run();

private:
    ros::NodeHandle n_;

    ros::Publisher twist_pub_;

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
int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle n;

    // ** Create object detection object
    Object_Detection o(n);

    // ** Run
    o.run();
    return 0;
}

Object_Detection::Object_Detection(const ros::NodeHandle &n)
    : n_(n), rgb_transport_(n), depth_transport_(n)
{
    // ** Publishers
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);

    // ** Subscribers
    rgb_sub_.subscribe(rgb_transport_,
                       "/camera/rgb/image_raw", QUEUE_SIZE);
    depth_sub_.subscribe(depth_transport_,
                       "/camera/depth/image", QUEUE_SIZE);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Object_Detection::RGBD_Callback, this, _1, _2));

}

void Object_Detection::run()
{
    // ** Publish data
    ros::Rate rate(PUBLISH_RATE); // 10 Hz

    while(ros::ok())
    {
        // ** Create msg
        geometry_msgs::Twist msg;

        // ** Compute control commands
//        control(msg.PWM1, msg.PWM2);
        // TO DO: image analysis
        msg.linear.x = 0.25;
        msg.angular.z = 0;

        // ** Publish
        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        rate.sleep();
    }

}

void Object_Detection::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                     const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;

    // ** Create binary mask
    cv::Mat m2(depth_img);
    m2.convertTo(m2, CV_32F);
    double max, min;
    cv::minMaxLoc(m2, &min, &max);
    cv::Mat mask(m2.rows, m2.cols, CV_8U);
    cv::threshold(m2, mask, DISTANCE_THRESHOLD_MULTIPLIER*min, 255, cv::THRESH_BINARY_INV);
    cv::imshow("MASK", mask);
    cv::waitKey(1);

    // ** Extract countour and mass center

//    //DILATE IMAGE
//         Mat img2;
//         dilate(img1,img2,Mat(),Point(-1,-1),20); //dilate image1 to remove single pixels and name it image2
//         //Mat() means standard kernal, 20 means iteration times

//         namedWindow("MyWindow2", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow2"
//         imshow("MyWindow2", img2); //display image2


//    waitKey(0); //wait infinite time for a keypress
//    destroyWindow("MyWindow2"); //destroy the window with the name, "MyWindow2"



    //FIND CONTOUR
    cv::Mat mask_conv;
    mask.convertTo(mask_conv, CV_8U);
    cv::Mat img3 = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    std::vector<std::vector<cv::Point> > contours, final_contours;
    std::vector<cv::Point> main_contour;
    cv::Scalar color( 0,0,255);//value for red, green and blue, 255,255,255 means white and 0,0,0 means black

    cv::findContours(mask_conv,contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Keep largest one
    double max_size = 0;
    int idx=-1;
    for(unsigned int i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > max_size)
        {
            max_size = area;
            idx = i;
        }
    }

    main_contour = contours[idx];
    final_contours.push_back(main_contour);

    //FIND CENTER POINT of largest contour

    cv::Moments mu = cv::moments(main_contour);


     //Mass center
     cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

    //PRINT MASS CENTER POINTS
    //notice there are two pair of center points because one pair is the center of the window,
    //you can probably use the difference of the values later for P-control

    std::cout << "Center (X,Y): "<< mc << std::endl;
    cv::drawContours(img3,final_contours,-1,color,0);  //-1 means draw all contours, 0 means single pixel thickness
    cv::circle(img3,mc,5,cv::Scalar(255,0,0),-1);
    cv::imshow( "Contours", img3 );
    cv::waitKey(1); //wait infinite time for a keypress

}

