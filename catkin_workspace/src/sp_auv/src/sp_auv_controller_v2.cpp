#define DEPTH_THURST_OFFSET -20
#define THRUST_LIMIT 70

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

float heading;
float roll;
float pitch;
float depth;
int count;

float des_depth;
float des_heading;
float des_thrust;

float depth_tolerance;
float roll_kp;
float depth_kp;
float heading_kp;
std::string test;

int clamp(int v, int minv, int maxv)
{
    return std::min(maxv,std::max(minv, v));
}


void CompassCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //ROS_INFO("x: [%f]", msg->x);
  //ROS_INFO("y: [%f]", msg->y);
  //ROS_INFO("theta: [%f]", msg->theta);
  roll = msg->x;
  pitch = msg->y;
  heading = msg->z;
}

void SounderCallback(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("depth: [%2.1f]", msg->data);
  depth = msg->data;
}

void DesDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_depth = msg->data;
}

void DesHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_heading = msg->data;
}

void DesThrustCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_thrust = msg->data;
}


int main(int argc, char **argv)
{


  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  std_msgs::Int16 thrust_hl, thrust_hr, thrust_vl, thrust_vr;

  ros::Subscriber sub_compass = n.subscribe("Sparton3DOF", 1000, CompassCallback);
  ros::Subscriber sub_sounder = n.subscribe("DepthSounder", 1000, SounderCallback);
  ros::Subscriber sub_des_dep = n.subscribe("DesiredDepth", 1000, DesDepthCallback);
  ros::Subscriber sub_des_hed = n.subscribe("DesiredHeading", 1000, DesHeadingCallback);
  ros::Subscriber sub_des_thr = n.subscribe("DesiredThrust", 1000, DesThrustCallback);
  ros::Publisher pub_hl = n.advertise<std_msgs::Int16>("thrust_hl", 500);
  ros::Publisher pub_hr = n.advertise<std_msgs::Int16>("thrust_hr", 500);
  ros::Publisher pub_vl = n.advertise<std_msgs::Int16>("thrust_vl", 500);
  ros::Publisher pub_vr = n.advertise<std_msgs::Int16>("thrust_vr", 500);
  
  ros::NodeHandle nh("~");
  double getval;

  nh.param("RollKP",getval,1.);
  roll_kp = (float) getval;
  ROS_ERROR("Out: %2.1f",roll_kp);

  nh.param("DepthKP",getval,-10.0);
  depth_kp = (float) getval;

  nh.param("HeadingKP",getval,3.0);
  heading_kp = (float) getval;


  ros::Rate loop_rate(10);

  //ros::spin();

  while (ros::ok()) {


    ros::spinOnce();

      // Depth Control
      float depth_E = depth - des_depth; //DESIRED_DEPTH;
      float vert_thrust;
      float roll_comp = roll * roll_kp;
      
      ROS_INFO("Depth Error: [%2.1f]", depth_E);

      //if ((depth_E*depth_E) > ( depth_tolerance * depth_tolerance )) 
      
      vert_thrust = (depth_kp * depth_E) + DEPTH_THURST_OFFSET;

      thrust_vl.data = clamp(vert_thrust+roll_comp,-THRUST_LIMIT,THRUST_LIMIT);
      thrust_vr.data = clamp(vert_thrust-roll_comp,-THRUST_LIMIT,THRUST_LIMIT);


      // Heading Control
      float heading_E = heading - des_heading ;//DESIRED_HEADING;
      float heading_D = (heading_kp * heading_E);


      thrust_hr.data =  clamp( des_thrust - heading_D,-THRUST_LIMIT,THRUST_LIMIT);
      thrust_hl.data =  clamp( des_thrust + heading_D,-THRUST_LIMIT,THRUST_LIMIT);


      ROS_INFO("Heading Error: [%2.1f]", heading_E);;
      ROS_INFO("HorL Thrust: [%d]", thrust_hl.data);
      ROS_INFO("HorR Thrust: [%d]", thrust_hr.data);
      ROS_INFO("Vert Thrust: [%d]", thrust_vr.data);

      pub_vr.publish(thrust_vr);
      pub_vl.publish(thrust_vl);
      pub_hr.publish(thrust_hr);
      pub_hl.publish(thrust_hl);
       
      

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