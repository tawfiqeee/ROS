/*
Singapore Polytechnic
Singapore Autonomous Underwater Vehicle Challenge 2014
Computer Vision System
Version : PRE ALPHA
*/

//Include Libraries
#include <iostream>
#include <vector>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point32.h"

//Declarations
using namespace cv;
int state = 0;
int ballrelease = 0;
int count = 0;


int main(int argc, char **argv)
{
    //SETUP
    /*
    ros::init(argc, argv, "auv_camera");
    ros::NodeHandle n;
    ros::Publisher camera_pub_bl = n.advertise<geometry_msgs::Point32>("cam_black", 1000);
    ros::Publisher camera_pub_rd = n.advertise<geometry_msgs::Point32>("cam_red", 1000);
    ros::Publisher camera_pub_yl = n.advertise<geometry_msgs::Point32>("cam_yellow", 1000);
    ros::Publisher camera_pub_state = n.advertise<std_msgs::Int16>("cam_state", 1000);
    ros::Rate loop_rate(10);

    std_msgs::Int16 cam_state;
    geometry_msgs::Point32 cam_black;
    geometry_msgs::Point32 cam_red;
    geometry_msgs::Point32 cam_yellow;
    */
    //Initialise Bottom Camera
    FILE * pfile;
    pfile = fopen("results.txt","w");
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
    Mat hsvbottomblack;
    Mat hsvbottomred;
    Mat hsvfront;
    CvMoments hsvblackMoment;
    CvMoments hsvyellowMoment;
    CvMoments hsvredMoment;
    namedWindow("Original Frame (Bottom)",1);
    namedWindow("HSV Frame (Bottom)",1);
    namedWindow("Original Frame (Front)",1);
    namedWindow("HSV Frame (Front)",1);
    //cam_state.data=0;
    //camera_pub_state.publish(cam_state);
    while(1)
    {
        if (state == 0)
        {
            Mat framebottom;
            cambottom >> framebottom; // Get a new frame from camera
            GaussianBlur(framebottom, framebottom, Size(7,7), 1.5, 1.5);
            cvtColor(framebottom, hsvbottom, CV_BGR2HSV);
            //Black
            inRange(hsvbottom, Scalar(0, 0, 0), Scalar(102, 194, 30),hsvbottomblack);
            hsvblackMoment = moments(hsvbottomblack);
            double moment10 = cvGetSpatialMoment(&hsvblackMoment, 1, 0); //X coordinates
            double moment01 = cvGetSpatialMoment(&hsvblackMoment, 0, 1); //Y coordinates
            double area = cvGetCentralMoment(&hsvblackMoment, 0, 0); //Sum of all white color pixels
            // if the area<1000, then it's probably not wanted
            if(area>1000)
            {
                // calculate the position of the ball
                int posX = moment10/area;
                int posY = moment01/area;
                circle(framebottom,cvPoint(posX,posY),5,cvScalar(0,255,255),4);
                printf("[Black] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                fprintf(pfile,"[Black] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                //cam_black.x=posX;
                //cam_black.y=posY;
                //camera_pub_bl.publish(cam_black);
            }
            //Red
            inRange(hsvbottom, Scalar(0, 70, 100), Scalar(5, 255, 255),hsvbottomred);
            hsvredMoment = moments(hsvbottomred);
            moment10 = cvGetSpatialMoment(&hsvredMoment, 1, 0); //X coordinates
            moment01 = cvGetSpatialMoment(&hsvredMoment, 0, 1); //Y coordinates
            area = cvGetCentralMoment(&hsvredMoment, 0, 0); //Sum of all white color pixels
            // if the area<1000, then it's probably not wanted
            if(area>1000)
            {
                // calculate the position of the ball
                int posX = moment10/area;
                int posY = moment01/area;
                circle(framebottom,cvPoint(posX,posY),5,cvScalar(255,255,255),4);
                printf("[Red] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                fprintf(pfile,"[Red] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                //cam_red.x=posX;
                //cam_red.y=posY;
                //camera_pub_rd.publish(cam_red);
                if (area>60000)
                {
                    state = 1;
                    //cam_state.data=1;
                    //camera_pub_state.publish(cam_state);
                }
            }
            imshow("Original Frame (Bottom)", framebottom);
            imshow("[Black] Thresholded Frame (Bottom)", hsvbottomblack);
            imshow("[Red] Thresholded Frame (Bottom)", hsvbottomred);
            if(waitKey(30) >= 0) break;
        }
        
        else
        {
            if (state == 1)
            {
                Mat framefront;
                camfront >> framefront; // Get a new frame from camera
                GaussianBlur(framefront, framefront, Size(7,7), 1.5, 1.5);
                cvtColor(framefront, hsvfront, CV_BGR2HSV);
                inRange(hsvfront, Scalar(17, 90, 90), Scalar(40, 255, 255),hsvfront);
                hsvyellowMoment = moments(hsvfront);
                double moment10 = cvGetSpatialMoment(&hsvyellowMoment, 1, 0); //X coordinates
                double moment01 = cvGetSpatialMoment(&hsvyellowMoment, 0, 1); //Y coordinates
                double area = cvGetCentralMoment(&hsvyellowMoment, 0, 0); //Sum of all white color pixels
                // if the area<500, then it's probably not wanted.
                if(area>500)
                {
                    // calculate the position of the ball
                    int posX = moment10/area;
                    int posY = moment01/area;
                    circle(framefront,cvPoint(posX,posY),5,cvScalar(0,0,0),4);
                    printf("[Yellow] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                    fprintf(pfile,"[Yellow] X Coordinate = %d. Y Coordinate = %d.\n", posX,posY);
                    //cam_yellow.x=posX;
                    //cam_yellow.y=posY;
                    //camera_pub_yl.publish(cam_yellow);
                }
                imshow("Original Frame (Front)", framefront);
                imshow("Thresholded Frame (Front)", hsvfront);
                if(waitKey(30) >= 0) break;
            }
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}