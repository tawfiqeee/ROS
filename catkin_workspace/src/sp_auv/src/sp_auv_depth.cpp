/***
* This example expects the serial port has a loopback on it.
*
* Alternatively, you could use an Arduino:
*
* <pre>
* void setup() {
* Serial.begin(<insert your baudrate here>);
* }
*
* void loop() {
* if (Serial.available()) {
* Serial.write(Serial.read());
* }
* }
* </pre>
*/

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

int run(int argc, char **argv)
{
  if(argc < 3) {
    cerr << "Usage: serial_read <serial port address> ";
    cerr << "<baudrate> [test string]" << endl;
    return 0;
  }

 ros::init(argc, argv, "talker");
 ros::NodeHandle n;
 ros::Publisher depth_pub = n.advertise<std_msgs::Float32>("DepthSounder", 1000);
 ros::Rate loop_rate(10);

  ros::NodeHandle nh("~");
  std::string getval;
  std::string def_port="/dev/ttyUSB5";
  std::string def_baud="9600";

  nh.param("port",getval,def_port);
  string port(getval);

  nh.param("baud",getval,def_baud);


  // Argument 1 is the serial port


  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(getval.c_str(), "%lu", &baud);
#else
  sscanf(getval.c_str(), "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;



  // Get the Test string
  int count = 0;
  string test_string;
  if (argc == 4) {
    test_string = argv[3];
  } else {
    test_string = "Testing.";
  }

  // Test the timeout, there should be 1 second between prints
  //cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
  while (ros::ok()) {


    string result = my_serial.readline();
    char tmptxt[20]="";
    std_msgs::String msg;
    std::stringstream ss;
    std_msgs::Float32 depth;

    std::size_t length = result.copy(tmptxt,6,0);
    tmptxt[length]='\0';
    
    depth.data = atof(tmptxt);

    ss << tmptxt ;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    depth_pub.publish(depth);

    ros::spinOnce();

    loop_rate.sleep();


    //count += 1;
  }


 // *****************************************************************************************




/*

  while (ros::ok()) {


    string result = my_serial.readline();
    char tmptxt[20]="";
    std_msgs::String msg;
    std::stringstream ss;

    std::size_t length = result.copy(tmptxt,6,0);
    tmptxt[length]='\0';
    
    ss << tmptxt ;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();


    //count += 1;
  }

  // Test the timeout at 250ms, but asking exactly for what was written
  count = 0;
  cout << "Timeout == 250ms, asking for exactly what was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length());

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms, but asking for 1 less than what was written
  count = 0;
  cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()-1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }
  */
  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
