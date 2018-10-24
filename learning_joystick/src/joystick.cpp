#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_x,linear_y,angular_z, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool pressed_6;
  ros::Timer timer_;

};

Teleop::Teleop():
  ph_("~"),
  linear_x(1),
  linear_y(4),
  angular_z(2),
  deadman_axis_(6),
  l_scale_(0.3),
  a_scale_(0.5)
{
  ph_.param("axis_linear_x", linear_x, linear_x);
  ph_.param("axis_linear_y", linear_y, linear_y);
  ph_.param("axis_angular", angular_z, angular_z);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  pressed_6 = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

  timer_ = ph_.createTimer(ros::Duration(0.1), boost::bind(&Teleop::publish, this));
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_ * joy->axes[angular_z];
  vel.linear.x = l_scale_ * joy->axes[linear_x];
  vel.linear.y = l_scale_ * joy->axes[linear_y];
  last_published_ = vel;
  pressed_6 = joy->buttons[deadman_axis_];
}

void Teleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (pressed_6)
  {
    vel_pub_.publish(last_published_);
  }
  else if(!pressed_6)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  Teleop turtlebot_teleop;

  ros::spin();
}
