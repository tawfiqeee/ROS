#define DEPTH_ST_1 0.75
#define THRUST_ST_2 40
#define THRUST_LIMIT 70

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


int main(int argc, char **argv)
{


  ros::init(argc, argv, "statemachine");

  ros::NodeHandle n;
  std_msgs::Int16 thrust_hl, thrust_hr, thrust_vl, thrust_vr;

  ros::Subscriber sub_omode = n.subscribe("OpMode", 1000, OpModeCallback);
  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sounder = n.subscribe("DepthSounder", 1000, SounderCallback);
  ros::Subscriber sub_cb = n.subscribe("cam_black", 1000, CamBlackCallback);
  ros::Subscriber sub_cr = n.subscribe("cam_red", 1000, CamRedCallback);
  ros::Subscriber sub_cy = n.subscribe("cam_yellow", 1000, CamYellowCallback);
  ros::Subscriber sub_cst = n.subscribe("cam_state", 1000, CamStateCallback);

  ros::Publisher pub_des_dep = n.advertise<std_msgs::Float32>("DesiredDepth", 500);
  ros::Publisher pub_des_hed = n.advertise<std_msgs::Float32>("DesiredHeading", 500);
  ros::Publisher pub_des_thr = n.advertise<std_msgs::Float32>("DesiredThrust", 500);;
  
  

  ros::NodeHandle nh("~");
  double getval;

  nh.param("DepthTolerance",getval,.1);
  depth_tolerance = (float) getval;
  ROS_ERROR("Out: %2.1f",depth_tolerance);

  nh.param("CamHeadKP",getval,0.1);
  camera_heading_gain = (float) getval;

  nh.param("HeadingKP",getval,3.0);
  heading_kp = (float) getval;


  ros::Rate loop_rate(5);

  //ros::spin();

  state = 1;
  des_thrust.data = 0;


  while (ros::ok()) {


    ros::spinOnce();



    switch (state)

    {
      case 1:
      
        if(op_mode=="GO")
        {
          des_heading.data = heading;
          state = 2;
        };

      break;

      case 2:
      
         des_depth.data = DEPTH_ST_1;
         if ( depth > (des_depth.data + depth_tolerance) && depth < (des_depth.data-depth_tolerance) ) 
         {
          des_thrust.data = THRUST_ST_2; 
          state = 3;
         }

      break;

      case 3:
      
      if (st_cam == "red") state = 4;

      if (pt_cb.x != 0) des_heading.data += pt_cb.x * camera_heading_gain * (depth / 320);



      break;

      case 4:
      
       //dropball();
      //

      break;

      case 5:
      
      // Accoustic;

      break;




    }

    pub_des_dep.publish(des_depth);
    pub_des_thr.publish(des_thrust);
    pub_des_hed.publish(des_heading);





















/*



      // Depth Control
 //     float depth_E = depth - des_depth; //DESIRED_DEPTH;
      
      ROS_INFO("Depth Error: [%2.1f]", depth_E);

      if ((depth_E*depth_E) > ( depth_tolerance * depth_tolerance )) 
      {
          thrust_vr.data=thrust_vl.data = clamp((depth_kp * depth_E) + DEPTH_THURST_OFFSET,-THRUST_LIMIT,THRUST_LIMIT);
          //pub_vr.publish(thrust_vr);
          //pub_vl.publish(thrust_vl);
          ROS_INFO("Vert Thrust: [%d]", thrust_vr.data);
      }

      // Heading Control
   //   float heading_E = heading - des_heading ;//DESIRED_HEADING;
      float heading_D = (heading_kp * heading_E);


     // thrust_hr.data =  clamp( des_thrust - heading_D,-THRUST_LIMIT,THRUST_LIMIT);
      thrust_hl.data =  clamp( des_thrust + heading_D,-THRUST_LIMIT,THRUST_LIMIT);


      ROS_INFO("Heading Error: [%2.1f]", heading_E);;
      ROS_INFO("HorL Thrust: [%d]", thrust_hl.data);
      ROS_INFO("HorR Thrust: [%d]", thrust_hr.data);

      //pub_hr.publish(thrust_hr);
      //pub_hl.publish(thrust_hl);
  */     
      

      loop_rate.sleep();

  }



  return 0;
}


/*
{


    ros::spinOnce();

      // Depth Control
      float depth_E = depth - DESIRED_DEPTH;

      if ((depth_E*depth_E) > (DEPTH_TOLERANCE*DEPTH_TOLERANCE) 
      {
          thrust_vr.data=thrust_vl.data = (DEPTH_KP * depth_E) + DEPTH_THURST_OFFSET;
          pub_vr.publish(thrust_vr);
          pub_vl.publish(thrust_vl);
      }

      // Heading Control
      float heading_E = heading - DESIRED_HEADING;
      if (heading_E > 0) 
      {
          thrust_hr.data =
          thrust_hl.data = DE(HEADING_KP * heading_E) 


          pub_vr.publish(thrust_hr);
          pub_vl.publish(thrust_hl);
      }      



    loop_rate.sleep();


    //count += 1;
  }

*/