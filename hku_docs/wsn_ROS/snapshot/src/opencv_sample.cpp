#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <cv.h>
//#include <highgui.h> // need this for imwrite()
#include <iostream>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

bool default_input;

class ImageConverter
{
  // member variables
  ros::NodeHandle nh_, priv_nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int image_num;

  public:
  ImageConverter()
    : nh_(),
      priv_nh_("~"),
      it_(nh_)
  {
    // We'll publish the resulting image on "out" and subscribe to "in"
    image_pub_ = it_.advertise("opencv_sample_out", 1);
    //default to topic: //multisense_sl/camera/right/image_rect_color
    if (default_input) {
       image_sub_ = it_.subscribe("//multisense_sl/camera/right/image_rect_color", 1, &ImageConverter::imageCb, this);
       ROS_INFO("using default input of //multisense_sl/camera/right/image_rect_color");
       ROS_INFO("to use an altenative input topic, run as: rosrun opencv_sample opencv_sample in:=//topic_name");
    }
    else
       image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
  }

  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      // Create a copy of the image in msg, requesting that the image be converted to BGR8 if it isn't already
      // We make a copy so that we can operate in place on it... if we wanted a read-only copy, we could use toCvShare to avoid extra memcpy ops
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   //do interesting work here...
    cv::dilate(cv_ptr->image, cv_ptr->image, cv::Mat(), cv::Point(-1,-1), 10);

    // Publish the modified image
    image_pub_.publish(cv_ptr->toImageMsg());
    trigger= 0;
  }
};

int main(int argc, char** argv)
{

  default_input=true; //use default input, unless command-line arg is provided
  if (argc==2)
    default_input=false;

  ros::init(argc, argv, "snapshot");
  savenum=0;
  ROS_INFO("wsn snapshot pgm: ");
  ROS_INFO("do: rosrun image_view image_view image:=/opencv_sample_out to see the output of this program");
  trigger=0;  // by default, don't save image
  ImageConverter ic;
  
  while (true)
  {
    ROS_INFO("enter 1 to process single step, 0 to quit:");
    std::cin>>trigger;
    if (trigger==0)
      return 0;
    ros::spinOnce();
  }
    return 0;
}
