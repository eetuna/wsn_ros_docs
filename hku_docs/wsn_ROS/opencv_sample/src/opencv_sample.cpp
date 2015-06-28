// example program using opencv
// by default, grabs image data from ROS topic //multisense_sl/camera/right/image_rect_color, though
//  the source is a command-line argument option
// This example single-steps with user input for a threshold
//  A hard-coded target color is defined in this code


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int thresh=30;
bool default_input;

cv::Vec3d target_color; // FILL IN THE VALUES OF THE TARGET COLOR IN main();  HARD CODED IN THIS EXAMPLE

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
    cv_bridge::CvImagePtr cv_ptr;  //use cv_ptr->image in place of "image" in OpenCV manual
   int dist; //color distance
   cv::Vec3d pixel_color; //variable to hold a pixel color for comparison

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
   // image dilation example:
   // cv::dilate(cv_ptr->image, cv_ptr->image, cv::Mat(), cv::Point(-1,-1), 10);


   // for each pixel
   int nl= cv_ptr->image.rows; // number of lines
   int nc= cv_ptr->image.cols;
   ROS_INFO("number of rows: %d; number of columns: %d",nl, nc);

  //NOTE: could/should wrap this up in a class and run faster using iterators
  // also, since this example only outputs black or white, could have used a gray-scale or binary image output
     // process each pixel ---------------------
  for (int i=0;i<nc;i++)
   for (int j=0;j<nl;j++) {
     // compute distance from target color
     //if (getDistance(*it)<minDist) {
        pixel_color[0]=cv_ptr->image.at<cv::Vec3b>(j,i)[0];
        pixel_color[1]=cv_ptr->image.at<cv::Vec3b>(j,i)[1];
        pixel_color[2]=cv_ptr->image.at<cv::Vec3b>(j,i)[2];
        dist = abs(pixel_color[0]-target_color[0])+ abs(pixel_color[1]-target_color[1])+ abs(pixel_color[2]-target_color[2]);

	cv_ptr->image.at<cv::Vec3b>(j,i)[0]= 255;
	cv_ptr->image.at<cv::Vec3b>(j,i)[1]= 0;
	cv_ptr->image.at<cv::Vec3b>(j,i)[2]= 0;
      
     if (dist>thresh) {
	cv_ptr->image.at<cv::Vec3b>(j,i)[0]= 0;
	cv_ptr->image.at<cv::Vec3b>(j,i)[1]= 0;
	cv_ptr->image.at<cv::Vec3b>(j,i)[2]= 0;
      }
     else {
	cv_ptr->image.at<cv::Vec3b>(j,i)[0]= 255;
	cv_ptr->image.at<cv::Vec3b>(j,i)[1]= 255;
	cv_ptr->image.at<cv::Vec3b>(j,i)[2]= 255;
      }
  }  //done with all pixels

    // Publish the modified image
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{

// SET THE TARGET COLOR HERE--keep pixels that are close to this color
   target_color[0]=30;
   target_color[1]=30;
   target_color[2]=30;

  default_input=true; //use default input, unless command-line arg is provided
  if (argc==2)
    default_input=false;

  ros::init(argc, argv, "opencv_sample");

  ROS_INFO("wsn opencv_sample pgm: ");
  ROS_INFO("do: rosrun image_view image_view image:=/opencv_sample_out to see the output of this program");

  ImageConverter ic;
  
  while (true)
  {
    ROS_INFO("enter threshold distance:  0 to quit:");
    std::cin>>thresh;
    if (thresh==0)
      return 0;
    ros::spinOnce();
  }
    return 0;
}
