/*

  revised March 4, wsn, to demo mouse interaction w/ image from point cloud
  subscribes to point cloud from robot
  converts to image
  displays image
  allows for mouse-click input
  need to extend this to use input info
  also, don't update images after mouse clicks?  (get single image?)

 * color_filter_example.cpp
 * 
 * This is an example of filtering data in a point cloud
 * via color and other image processing techniques
 */

// Standard ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// PCL includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//geometry messages
#include <geometry_msgs/Vector3.h>


#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

// Function prototypes here
sensor_msgs::PointCloud2 filter_rgb(const sensor_msgs::PointCloud2ConstPtr& input_cloud,
                                                                                        int l_r,
                                                                                        int u_r,
                                                                                        int l_g,
                                                                                        int u_g,
                                                                                        int l_b,
                                                                                        int u_b);

// Global variables here
ros::Publisher cloud_pub; // will publish the cloud
ros::Publisher image_pub; // publish cloud converted to image
sensor_msgs::Image ROS_image_of_input_cloud;

int mouse_click_x=0;
int mouse_click_y=0;
float cloudPtX;
float cloudPtY;
float cloudPtZ;
int  cloudPtR;
int  cloudPtG;
int  cloudPtB;

bool getPtFlag=false;
bool firstCloud=true;
bool pickedTarget=false;

Mat img;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& ros_input_cloud)
{
        int pcl_index;
        sensor_msgs::PointCloud2 ros_output_cloud;

        if (firstCloud) {
          // convert ROS input cloud to PCL format
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          // convert Input cloud from ROS format to PCL format
          pcl::fromROSMsg(*ros_input_cloud, *pcl_input_cloud);
          // convert input cloud to ROS image and publish it:
	  pcl::toROSMsg (*ros_input_cloud, ROS_image_of_input_cloud); //convert the cloud
          image_pub.publish(ROS_image_of_input_cloud);
          firstCloud=false;
        }

        if(getPtFlag) {
          getPtFlag=false; // turn off trigger

          // Input cloud in PCL format
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          // convert Input cloud from ROS format to PCL format
          pcl::fromROSMsg(*ros_input_cloud, *pcl_input_cloud);
          //ROS_INFO("width of cloud: %d ",pcl_input_cloud->width);
          //ROS_INFO("height of cloud: %d",pcl_input_cloud->height);

          // here is an example of color filtering of a point cloud:
          // currently only allowing near black colors through--hard-coded filter bounds
          ros_output_cloud = filter_rgb(ros_input_cloud,
                                                                  0, 47,  // red
                                                                  0, 42,  // green
                                                                  0, 52); // blue
  
          cloud_pub.publish(ros_output_cloud);
 



          // convert input cloud to ROS image and publish it:
	  pcl::toROSMsg (*ros_input_cloud, ROS_image_of_input_cloud); //convert the cloud
          image_pub.publish(ROS_image_of_input_cloud);

          // problem here...really want to reference mouse click to PRIOR point cloud...
          // the one used to create the display for the received mouse click

	  pcl_index =  mouse_click_y * (pcl_input_cloud->width) + mouse_click_x;
          // look up 3-D point info corresponding to mouse-click coords:
                cloudPtX = pcl_input_cloud->points[pcl_index].x;
                cloudPtY = pcl_input_cloud->points[pcl_index].y;
                cloudPtZ = pcl_input_cloud->points[pcl_index].z;
          ROS_INFO("selected point x,y,z = %f %f %f",cloudPtX,cloudPtY,cloudPtZ);
		cloudPtR = pcl_input_cloud->points[pcl_index].r;
		cloudPtG = pcl_input_cloud->points[pcl_index].g;
		cloudPtB = pcl_input_cloud->points[pcl_index].b;
          ROS_INFO("color of pt, RGB = %d %d %d",cloudPtR,cloudPtG,cloudPtB);
          // should publish target..will need to convert from left-camera optical frame

          // recognize new target only if data is valid:
          if (cloudPtX==cloudPtX && cloudPtY==cloudPtY && cloudPtZ==cloudPtZ)
             pickedTarget=true;

        }
	  
}



// callback function for mouse events
void onMouse(int event, int x, int y, int flags, void* param)
{
    char text[20];
    Mat img2, img3;
    cv_bridge::CvImagePtr cv_ptr;  //use cv_ptr->image in place of "image" in OpenCV manual

    cv_ptr = cv_bridge::toCvCopy(ROS_image_of_input_cloud, enc::BGR8);

    img = cv_ptr->image; // ????

    img2 = img.clone();
    //img2 = cv_ptr->image;

    if (event == CV_EVENT_LBUTTONDOWN)
    {
        mouse_click_x = x;  //pass mouse coords as globals
        mouse_click_y = y;
        Vec3b p = img2.at<Vec3b>(y,x);
        sprintf(text, "R=%d, G=%d, B=%d", p[2], p[1], p[0]);
	ROS_INFO("image RGB = %d %d %d",p[2],p[1],p[0]);
        ROS_INFO("image x,y = %d %d",mouse_click_x,mouse_click_y);
        getPtFlag=true;

    }
    else if (event == CV_EVENT_RBUTTONDOWN)
    {
        cvtColor(img, img3, CV_BGR2HSV);
        Vec3b p = img3.at<Vec3b>(y,x);
        sprintf(text, "H=%d, S=%d, V=%d", p[0], p[1], p[2]);
    }
    else
        sprintf(text, "x=%d, y=%d", x, y);


    putText(img2, text, Point(5,15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    imshow("image_window", img2);


        
}




sensor_msgs::PointCloud2 filter_rgb(const sensor_msgs::PointCloud2ConstPtr& ros_input_cloud,
                                                                        int l_r,
                                                                        int u_r,
                                                                        int l_g,
                                                                        int u_g,
                                                                        int l_b,
                                                                        int u_b)
{
        // Input cloud in PCL format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Filtered cloud in PCL format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        sensor_msgs::PointCloud2 ros_filtered_cloud; // Filtered Cloud in ROS Format

        // convert Input cloud from ROS format to PCL format
        pcl::fromROSMsg(*ros_input_cloud, *pcl_input_cloud);

        pcl_filtered_cloud->width = pcl_input_cloud->width;
        pcl_filtered_cloud->height = pcl_input_cloud->height;
        pcl_filtered_cloud->points.resize(pcl_filtered_cloud->width * pcl_filtered_cloud->height);

        for(unsigned int i = 0; i < pcl_input_cloud->size(); ++i)
        {
                if(pcl_input_cloud->points[i].r < l_r || pcl_input_cloud->points[i].r > u_r)
                        continue;
                if(pcl_input_cloud->points[i].g < l_g || pcl_input_cloud->points[i].g > u_g)
                        continue;
                if(pcl_input_cloud->points[i].b < l_b || pcl_input_cloud->points[i].b > u_b)
                        continue;

                // if made it to here then the point is within all 3 color ranges
                // so copy the point to the filtered cloud
                pcl_filtered_cloud->points[i].x = pcl_input_cloud->points[i].x;
                pcl_filtered_cloud->points[i].y = pcl_input_cloud->points[i].y;
                pcl_filtered_cloud->points[i].z = pcl_input_cloud->points[i].z;
                pcl_filtered_cloud->points[i].r = pcl_input_cloud->points[i].r;
                pcl_filtered_cloud->points[i].g = pcl_input_cloud->points[i].g;
                pcl_filtered_cloud->points[i].b = pcl_input_cloud->points[i].b;
        }

        // convert Filtered cloud from PCL format to ROS format
        pcl::toROSMsg(*pcl_filtered_cloud, ros_filtered_cloud);

        ros_filtered_cloud.header = ros_input_cloud->header;
        
        return ros_filtered_cloud;
}

int main(int argc, char** argv)
{
        ros::Publisher handGoalPub;
        geometry_msgs::Vector3 handTarget;
        ros::init(argc, argv, "color_filter_example");
        ros::NodeHandle nh_processed,nh_orig_image,nh_handGoal;
        image_transport::ImageTransport it(nh_orig_image); // fancy class used for dealing with image topics
        ROS_INFO("subscribing to multisense points2...");
  
        // subscribe to this pointcloud2 topic
        ros::Subscriber sub = nh_orig_image.subscribe("/multisense_sl/camera/points2", 1, cloud_callback);
        
        // convert the incoming, original pointcloud into an image and publish it to this topic:
        image_pub = nh_orig_image.advertise<sensor_msgs::Image>(ros::this_node::getName() + "/cloud2image", 1);
 
        // publish this processed point-cloud topic to this topic
        cloud_pub = nh_processed.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/cloud", 1);

        // publish selected point:
        handGoalPub = nh_handGoal.advertise<geometry_msgs::Vector3>("handGoal",1);


        cv_bridge::CvImagePtr cv_ptr;  //use cv_ptr->image in place of "image" in OpenCV manual example


        ROS_INFO("getting image..."); // need to receive an image from callback before can display it;
                                      // should test for success rather than depend on luck and timing
        for (int i=0;i<10;i++) {
           ros::spinOnce(); // let the callback get an image;
           ros::Duration(0.1).sleep();
        }

 
        // convert the resulting ROS image method into an open-cv image
        cv_ptr = cv_bridge::toCvCopy(ROS_image_of_input_cloud, enc::BGR8);

        img = cv_ptr->image; // Mat img is now synonymous with cv_ptr->image

        ROS_INFO("setting up named window");

        namedWindow("image_window");
        
        // assign "onMouse()" as the callback fnc for mouse events
        setMouseCallback("image_window", onMouse, 0);
        ROS_INFO("ready to display...");

        // odd logic here;
        while(ros::ok()) {
          ros::spinOnce(); // try to get a fresh pointcloud received;
          imshow("image_window", img);
          waitKey(100); // images update only after mouse events;

          // decide if should publish a point:
          if (pickedTarget) { //publish target
		handTarget.x=cloudPtX;
		handTarget.y=cloudPtY;
		handTarget.z=cloudPtZ;
               
               //convert from left-camera optical frame to pelvis frame
               //XXX

               // and publish the result
               handGoalPub.publish(handTarget);



          }
          ros::spinOnce();
        }
}
