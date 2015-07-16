/*
 * hand_control.cpp
 *
 *  Created on: 2 Apr 2015
 *      Author: eduardo
 */
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <dynamixel_msgs/JointState.h>


class HandController
{
public:
	ros::NodeHandle nh_;
	ros::Publisher position_pub_;
	ros::Subscriber mouse_sub_;
	ros::Subscriber servo_sub_;
	bool close;
	bool opening;
	bool stop;
	float hand_load;
	float hand_pos;
	bool confirm;
  HandController(){
	  	position_pub_ = nh_.advertise <std_msgs::Float64>("/tilt_controller/command", 1);
	  	mouse_sub_ = nh_.subscribe <std_msgs::Int8>("/hand", 10, &HandController::handCallback, this);
	  	servo_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state",10,&HandController::handServo,this);
	  	close=false;
	  	opening=false;
	  	stop=false;
	  	hand_load=0;
	  	hand_pos=0;
	  	confirm=false;
  }

  static float map(float x,float in_min,float in_max,float out_min,float out_max)
  {
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void positionSND(std_msgs::Float64 pos_msg, double duration){
	  ros::Rate loop_rate(50);
	  double time_msg=0;
	  double time_start=(double)ros::Time::now().toSec();
	  //ROS_INFO("Moving...");
	  while (time_msg< time_start+duration) /* Send command for duration seconds*/
	  {
	      position_pub_.publish(pos_msg);
		  time_msg=(double)ros::Time::now().toSec();
		  ros::spinOnce();
		  loop_rate.sleep();
	  }//time loop
  }

  void handCallback(const std_msgs::Int8::ConstPtr& mouse)
  {
	  close=confirm=opening=false;
	  if(mouse->data==1)
		  close=true;
	  if (mouse->data==3)
		  opening=true;
	  if (mouse->data==2)
		  confirm=true;
  }
  void handServo(const dynamixel_msgs::JointState::ConstPtr& data)
  {
	  hand_load=fabs(data->load);
	  if(hand_load>0.3)
		  if(!data->is_moving)
			  stop=true;
	  hand_pos=data->current_pos;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_control");
  HandController hand;

  ros::Rate loop_rate(50);
  std_msgs::Float64 msg_pos;
  ROS_INFO("Loading position parameters from server ...");
  ros::param::set("/tilt_controller/motor/max",1024);
  ROS_INFO("ok max");
  ros::param::set("/tilt_controller/motor/min",0);
  ROS_INFO("ok min");
  	//Set motor to zero
  	float max_angle_deg=150;   //Ax-12 has a rotation about 300 degrees (150+/150-)
  	float max_angle=max_angle_deg*M_PI/180;
  	float zero=0.0;
  	float position,internal_counter=0.0;
  	ROS_INFO("Min angle: %f Max angle: %f",-max_angle,max_angle);
  	ROS_INFO("Sending servo to %f",zero);
  	msg_pos.data=zero;
  	hand.positionSND(msg_pos,1);
  	while(ros::ok())
  	{
  		if(hand.close)
  		{
  			if(hand.stop)
  				ROS_INFO("HIGH LOAD");
  		  	else
  		  	{
  		  		//position=hand.map(hand.hand_pos,0,1023,-max_angle,max_angle);
  		  		position=hand.hand_pos;
  		  		internal_counter=position+0.15;
  		  		if (internal_counter>max_angle) internal_counter=max_angle;
  		  		if (internal_counter<-max_angle) internal_counter=-max_angle;
  		  		ROS_INFO("Load: %f",hand.hand_load);
  		  		ROS_INFO("Position current: %f goal: %f",hand.hand_pos,internal_counter);
 		  		msg_pos.data=internal_counter;
  		  		hand.positionSND(msg_pos,0.1);
  		  	}
  		}
  		if(hand.opening)
  		{
//  			hand.opening=false;
//  			hand.stop=false;
  			position=hand.hand_pos;
  			internal_counter=position-0.15;
  			if (internal_counter>max_angle) internal_counter=max_angle;
  			if (internal_counter<-max_angle) internal_counter=-max_angle;
  			ROS_INFO("Load: %f",hand.hand_load);
  			ROS_INFO("Position current: %f goal: %f",hand.hand_pos,internal_counter);
  			msg_pos.data=internal_counter;
  			hand.positionSND(msg_pos,0.1);
  		}
  		ros::spinOnce();
  		loop_rate.sleep();
  	}
  	return 0;
}
