/*
 * mouse_teleop.cpp
 *
 *  Created on: 2 Apr 2015
 *      Author: eduardo
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>


class MouseTeleop
{
public:
  MouseTeleop(){
	  	vel_pub_ = nh_.advertise<std_msgs::Int8>("/hand", 1);
	  	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &MouseTeleop::joyCallback, this);
  };
  ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    std_msgs::Int8 msg;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

};

void MouseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	msg.data=0;
	if(joy->buttons[0])
	{
		//left
		 msg.data=1;
	}
	else
	{
		if(joy->buttons[1])
		{
			//middle
			msg.data=2;
		}
		else
		{
			if(joy->buttons[2])
				msg.data=3;
		}
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mouse");
  MouseTeleop teleop;
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
	  	  	  teleop.vel_pub_.publish(teleop.msg);
  	  	  	  ros::spinOnce();
  			  loop_rate.sleep();
  }
  return 0;
}


