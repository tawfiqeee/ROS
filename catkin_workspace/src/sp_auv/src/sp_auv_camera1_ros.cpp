/*
Singapore Polytechnic
Singapore Autonomous Underwater Vehicle Challenge 2014
Computer Vision System
Version : PRE ALPHA
*/

// ########### ROS stuff ##########################

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

// ################################################

//Include Libraries
#include "opencv2/opencv.hpp"

//Declarations
using namespace cv;
int state = 0;
int ballrelease = 0;
int count = 0;

int main(int argc, char **argv)
{
    // ########### ROS stuff ##########################
/*
    ros::init(argc, argv, "auv_camera");
    ros::NodeHandle n;
    ros::Publisher camera_pub_bl = n.advertise<geometry_msgs::Point32>("cam_black", 1000);
    ros::Publisher camera_pub_rd = n.advertise<geometry_msgs::Point32>("cam_red", 1000);
    ros::Publisher camera_pub_yl = n.advertise<geometry_msgs::Point32>("cam_yellow", 1000);
    ros::Publisher camera_pub_state = n.advertise<std_msgs::String>("cam_state", 1000);
    ros::Rate loop_rate(10);

    std_msgs::String cam_state;
    geometry_msgs::Point32 cam_black;
    geometry_msgs::Point32 cam_red;
    geometry_msgs::Point32 cam_yellow;

    unsigned int i =0;
*/
    // ################################################   


    //SETUP
    //Initialise Bottom Camera
    VideoCapture cambottom(0); 
    if(!cambottom.isOpened()) //Checking sequence 
    {
        return -1;
    }
    cambottom.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cambottom.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    
    
    //Initialise Front Camera
    VideoCapture camfront(1);
    
    if(!camfront.isOpened()) //Checking sequence
    {
        return -1;
    }
    camfront.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    camfront.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    
    
    Mat hsvbottom;
    Mat hsvfront;
    namedWindow("Original Frame (Bottom)",1);
    namedWindow("Original Frame (Front)",1);
    for(;;)
    {
        Mat framebottom;
        cambottom >> framebottom; // get a new frame from camera

        cvtColor(framebottom, hsvbottom, CV_BGR2HSV);
        GaussianBlur(hsvbottom, hsvbottom, Size(7,7), 1.5, 1.5);
        imshow("Original Frame (Bottom)", hsvbottom);

       
        Mat framefront;
        camfront >> framefront; // get a new frame from camera
        cvtColor(framefront, hsvfront, CV_BGR2HSV);
        GaussianBlur(hsvfront, hsvfront, Size(7,7), 1.5, 1.5);
        imshow("Original Frame (Front)", hsvfront);
        if(waitKey(30) >= 0) break;

                    
        // ########### ROS stuff ##########################
/*
        cam_state.data="black";
        cam_black.x=0 + i++;
        cam_black.y=0+i;
        cam_red.x=1+i;
        cam_red.y=1+i;
        cam_yellow.x=2+i;
        cam_yellow.y=2+i;

        camera_pub_bl.publish(cam_black);
        camera_pub_rd.publish(cam_red);
        camera_pub_yl.publish(cam_yellow);
        camera_pub_state.publish(cam_state);
        ros::spinOnce();
        loop_rate.sleep();
*/
        // ################################################  
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}