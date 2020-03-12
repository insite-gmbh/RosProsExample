/*
 * rosserial Publisher/Subscriber Example for VEX Cortex
*/

#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"
#include "ros_lib/geometry_msgs/Twist.h"

#define LEFT_WHEELS_PORT 1
#define RIGHT_WHEELS_PORT 10

static int subscriberCalls = 0;
static int lastReportedSubscriberCalls = 0;
static bool clsreq = false;

void handleControl(const geometry_msgs::Twist& t){
  // power motors using the message event!
  // (default configuration is for a clawbot set up in the standard fashion).
  pros::Motor left_wheels (LEFT_WHEELS_PORT);
  pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
  left_wheels.move(-(int) (100 * t.linear.x + 50 * t.angular.z));
  right_wheels.move((int) (100 * t.linear.x - 50 * t.angular.z));
  subscriberCalls++;
}

void cls()
{
  clsreq = true;
}

// called from opcontrol.cpp
void setup()
{
  // make a node handle object and a subscriber for the Twist message.
  ros::NodeHandle  nh;
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel\0", &handleControl);
  std_msgs::String str_msg;
  ros::Publisher chatter("chatter\0", &str_msg);

  pros::lcd::initialize();
  pros::lcd::set_text(0, "running");

  // set up rosserial, and prepare to publish the chatter message
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  char* msg = (char*) malloc(100 * sizeof(char));
  pros::lcd::initialize();
  pros::lcd::register_btn0_cb(cls);

  int addDelayCounter = 0;
  int soret;
  int last_so_err = ros::SPIN_OK;
  // message data variable.
  while (1) {
    addDelayCounter++;
    if (!(addDelayCounter % 50) || subscriberCalls != lastReportedSubscriberCalls) {
      // send a message about the time!
      sprintf(msg, "[%d] Subscriber called %d times! last_so_err %d",
        (int) pros::c::millis(), subscriberCalls, last_so_err);
      last_so_err = ros::SPIN_OK;
      str_msg.data = msg;
      chatter.publish( &str_msg );
      lastReportedSubscriberCalls = subscriberCalls;
    }
    soret = nh.spinOnce();
    if (soret != ros::SPIN_OK && last_so_err == ros::SPIN_OK)
    {
      last_so_err = soret;
    }
    if (clsreq)
    {
      pros::lcd::clear();
      clsreq = false;
    }
    pros::c::delay(20);
  }
}
