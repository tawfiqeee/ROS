#define DEPTH_ST_1 0.75
#define THRUST_ST_2 40
#define THRUST_ST_4 40
#define THRUST_ST_5 50
#define THRUST_LIMIT 70
#define MATH_PI 3.141592

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

float heading;
float depth;
int count;
int state;

geometry_msgs::Point32 pt_cb, pt_cr, pt_cy;
std::string st_cam;
std::string op_mode;

std_msgs::Float32 des_depth,des_heading,des_thrust;

float depth_tolerance;
float depth_kp;
float heading_kp;
float camera_heading_gain;
float original_heading;
float time_out;
double time_since_go;


int clamp(int v, int minv, int maxv)
{
    return std::min(maxv,std::max(minv, v));
}

void OpModeCallback(const std_msgs::String::ConstPtr& msg)
{
  op_mode = msg->data;
}

void CompassCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  heading = msg->theta;
}

void SounderCallback(const std_msgs::Float32::ConstPtr& msg)
{;
  depth = msg->data;
}

void CamBlackCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  pt_cb.x = msg->x;
  pt_cb.y = msg->y;
}

void CamRedCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  pt_cr.x = msg->x;
  pt_cr.y = msg->y;;
}

void CamYellowCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  pt_cy.x = msg->x;
  pt_cy.y = msg->y;
}

void CamStateCallback(const std_msgs::String::ConstPtr& msg)
{
  st_cam = msg->data;
}

void TimerCallback(const ros::TimerEvent& event)
{
  time_since_go += 0.1; 
}


int main(int argc, char **argv)
{


  ros::init(argc, argv, "statemachine");

  ros::NodeHandle n;
  std_msgs::Int16 ball_drop;

  ros::Subscriber sub_omode = n.subscribe("OpMode", 1000, OpModeCallback);
  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sounder = n.subscribe("DepthSounder", 1000, SounderCallback);
  ros::Subscriber sub_cb = n.subscribe("cam_black", 1000, CamBlackCallback);
  ros::Subscriber sub_cr = n.subscribe("cam_red", 1000, CamRedCallback);
  ros::Subscriber sub_cy = n.subscribe("cam_yellow", 1000, CamYellowCallback);
  ros::Subscriber sub_cst = n.subscribe("cam_state", 1000, CamStateCallback);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), TimerCallback);
  ros::Publisher pub_des_dep = n.advertise<std_msgs::Float32>("DesiredDepth", 500);
  ros::Publisher pub_des_hed = n.advertise<std_msgs::Float32>("DesiredHeading", 500);
  ros::Publisher pub_des_thr = n.advertise<std_msgs::Float32>("DesiredThrust", 500);;
  ros::Publisher pub_ball_drop = n.advertise<std_msgs::Int16>("ball_drop", 500);


  

  ros::NodeHandle nh("~");
  double getval;

  nh.param("DepthTolerance",getval,.1);
  depth_tolerance = (float) getval;
  ROS_ERROR("Out: %2.1f",depth_tolerance);

  nh.param("CamHeadKP",getval,0.1);
  camera_heading_gain = (float) getval;

  nh.param("TimeOut",getval,600.0);
  time_out = getval;


  ros::Rate loop_rate(5);

  //ros::spin();

  state = 1;
  des_thrust.data = 0;


  while (ros::ok()) {


    ros::spinOnce();



    switch (state)

    {
      case 1:
        time_since_go = 0;
        if(op_mode=="GO" || op_mode=="QUAL")
        {
          des_heading.data = heading;
          original_heading = heading;
          des_depth.data = DEPTH_ST_1;
          state = 2;
        }

      break;

      case 2:
      
         if ( depth > (des_depth.data + depth_tolerance) && depth < (des_depth.data - depth_tolerance) ) 
         {
          des_thrust.data = THRUST_ST_2; 
          if(op_mode=="QUAL") state = 55;
          state = 3;
         }

      break;

      case 3:
      
      if (st_cam == "red") state = 4;

      if (pt_cb.x != 0) des_heading.data += pt_cb.x * camera_heading_gain * ((depth+0.05) / 320);

      break;

      case 4:
      
        ball_drop.data = 1;
        pub_ball_drop.publish(ball_drop);
        des_heading.data = original_heading + (MATH_PI / 2);
        des_thrust.data = THRUST_ST_4;
        state = 5;

      break;

      case 5:
      
      if (st_cam == "done") 
        {
          state = 6;
          des_heading.data = original_heading + MATH_PI;
          des_thrust.data = THRUST_ST_5;
        }
      if (pt_cy.x != 0) des_heading.data += pt_cy.x * camera_heading_gain * ((depth+0.05) / 320);

      break;

      case 6:
      
      //Acostics ???;

      break;

     case 55:
      
      //Qualifying, what else to do?

      break; 




    }
    ROS_INFO("State    : [%d]", state);
    ROS_INFO("Time(s)  : [%2.1f]", time_since_go);
    ROS_INFO("Heading  : [%2.1f]",heading);
    ROS_INFO("Depth    : [%2.1f]", depth);
    ROS_INFO("Des Thr  : [%2.0f]",des_thrust.data);

    if ((int)time_since_go > time_out)
    {
      des_depth.data = 2.5;
      des_thrust.data = 0;
      exit(0);
    }

    pub_des_dep.publish(des_depth);
    pub_des_thr.publish(des_thrust);
    pub_des_hed.publish(des_heading); 
      
    loop_rate.sleep();

  }



  return 0;
}

