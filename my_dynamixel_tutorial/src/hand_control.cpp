/*
 * hand_control.cpp
 *
 *  Created on: 2 Apr 2015
 *      Author: eduardo
 */



/*
 * mouse_teleop.cpp
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
  HandController(){
	  	position_pub_ = nh_.advertise <std_msgs::Float64>("/tilt_controller/command", 1);
	  	mouse_sub_ = nh_.subscribe <std_msgs::Int8>("/hand", 10, &HandController::handCallback, this);
	  	servo_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state",10,&HandController::handServo,this);
	  	close=false;
	  	opening=false;
	  	stop=false;
	  	hand_load=0;
	  	hand_pos=0;
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
	  close=false;
	  if(mouse->data==1)
		  close=true;
	  if (mouse->data==3)
		  opening=true;
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
  double min,max,init;
  ROS_INFO("Loading position parameters from server ...");
  	if(ros::param::get("/tilt_controller/motor/max",max))
  	{
  		ROS_INFO("ok max");
  	}
  	if(ros::param::get("/tilt_controller/motor/min",min))
  	{
  		ROS_INFO("ok min");
  	}
  	if(ros::param::get("/tilt_controller/motor/init",init))
  	{
  		ROS_INFO("ok init");
  	}
  	ROS_INFO("Encoder Min: %d  Max: %d  Init: %d  ",int(min),int(max),int(init));
  	float K=3;
    float action_threshold=0.02; //Radians
  	//Set motor to zero
  	float max_angle_deg=150;   //Ax-12 has a rotation about 300 degrees (150+/150-)
  	float max_angle=max_angle_deg*M_PI/180;
  	float relative_zero=hand.map(init,0,1023,-max_angle,max_angle);  //Zero position in radians from config file.
  	float relative_max=hand.map(max,0,1023,-max_angle,max_angle);
  	float relative_min=hand.map(min,0,1023,-max_angle,max_angle);
  	ROS_INFO("Radians Min: %f  Max: %f  Init: %f",relative_min,relative_max,relative_zero);
  	//First position equals relative zero
  	float position= relative_zero;
  	double max_rad=hand.map(max,0,1023,-max_angle,max_angle);
  	double min_rad=hand.map(min,0,1023,-max_angle,max_angle);
  	float internal_counter=position;
  	int old_position=position;
  	ROS_INFO("Sending servo to %f",position);
  	msg_pos.data=position;
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
  		  		internal_counter=position+0.1;
  		  		if (internal_counter>max_rad) internal_counter=max_rad;
  		  			if (internal_counter<min_rad) internal_counter=min_rad;
  		  				ROS_INFO("Load: %f",hand.hand_load);
  		  				ROS_INFO("Position current: %f goal: %f",hand.hand_pos,internal_counter);
 		  				msg_pos.data=internal_counter;
  		  				hand.positionSND(msg_pos,0.1);
  		  	}
  		}
  		if(hand.opening)
  		{
  			hand.opening=false;
  			hand.stop=false;
  			ROS_INFO("Opening");
  			position=internal_counter=relative_zero;
  			msg_pos.data=position;
  			hand.positionSND(msg_pos,0.3);
  		}
  			ros::spinOnce();
  			loop_rate.sleep();
  	}
  	return 0;
}
