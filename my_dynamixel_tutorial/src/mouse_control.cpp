#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include <sys/types.h>
#include <cstdlib>
#include <iostream>
#include <cmath>
int move_dir,servo;
int open_hand=0;
bool opening=0;

class MouseListener
{
	ros::NodeHandle nh_;
	ros::Publisher button_pub;
	int fd;
	public:
		MouseListener()
		{
			// Publish mouse button information
			//button_pub = nh_.advertise<std_msgs::Int16>("mouse",1);
			//Pointer to mouse device
			const char *pDevice = "/dev/input/event3";
			// Open Mouse
		    fd = open(pDevice, O_RDONLY);
		    int saved_flags = fcntl(fd, F_GETFL);
		    fcntl(fd, F_SETFL, saved_flags & ~O_NONBLOCK);
		    if(fd == -1)
		    {
		        printf("ERROR Opening mouse %s\n", pDevice);
		        //ros::shutdown();
		        exit(1);
		    }
		    button_pub = nh_.advertise <std_msgs::Float64>("/tilt_controller/command", 1);
		}
	
		~MouseListener()
		{
			ROS_INFO("Exiting");
		}

	int getMouseBtn()
	{

		struct input_event event;
		int bytes= read (fd,&event,sizeof(event));
		//std::cout<<" read";
		int increment=0;
        if(bytes > 0)
        {
        	switch(event.code)
        	{
        		case BTN_LEFT:
        			if(event.value>0)
        			{
        				ROS_INFO("DOWN");
        				open_hand=1;
        			}
        			else
        				ROS_INFO("UP");
        			break;
        		case BTN_RIGHT:
        			if(event.value>0)
        				{
        					opening=!opening;
        				}
        			else
        			{
        					ROS_INFO("Released");
        			}
        			break;
        		case REL_WHEEL:
        			if (event.value>0)
        			{
        				increment=event.value;
        			}
        			else
        			{
        				increment=event.value;
        			}
    				break;
        	}
        }
        return increment;
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
			button_pub.publish(pos_msg);
			time_msg=(double)ros::Time::now().toSec();
			ros::spinOnce();
			loop_rate.sleep();
		}//time loop
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mouse_listener");
	MouseListener ml;
	ros::Rate loop_rate(50);
	double min,max,init;
	std::string max_st, min_st,init_st;
	//Reading max,min and init positions from ROS param server, loaded from yaml file
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
	ROS_INFO("Min: %d  Max: %d  Init:%d  ",int(min),int(max),int(init));
	std_msgs::Float64 msg_pos;
	float K=0.05;
	float action_threshold=0.02; //Radians
	//Set motor to zero
	float max_angle_deg=150;   //Ax-12 has a rotation about 300 degrees (150+/150-)
	float max_angle=max_angle_deg*M_PI/180;
	float relative_zero=ml.map(init,0,1023,-max_angle,max_angle);  //Zero position in radians from config file.
	ROS_INFO("Abs 0 = %f   Rel 0= %f",ml.map(512,0,1023,-max_angle,max_angle),ml.map(init,0,1023,-max_angle,max_angle));
	//First position equals relative zero
	float position= relative_zero;
	double max_rad=ml.map(max,0,1023,-max_angle,max_angle);
	double min_rad=ml.map(min,0,1023,-max_angle,max_angle);
	float internal_counter=position;
	int old_position=position;
	ROS_INFO("Sending servo to %f",ml.map(position,0,1023,-max_angle,max_angle));
	msg_pos.data=position;
	ml.positionSND(msg_pos,1);
	while(ros::ok())
	{
			//Read relative increment in mouse wheel
			//std::cout<<"In \n";
			internal_counter+=K*ml.getMouseBtn();
			//std::cout<<"Out \n";
			//Check for max and min possible position
			position=internal_counter;
			if (internal_counter>max_rad) position=internal_counter=max_rad;
			if (internal_counter<min_rad) position=internal_counter=min_rad;
			if (fabs((old_position-position))>action_threshold)
			{
				//ROS_INFO("Encoder: %d  Angle: %f",position,current_radians-target_radians);
				if(open_hand) {position=internal_counter=relative_zero;open_hand=0;}
				old_position=position;
				msg_pos.data=position;
				ml.positionSND(msg_pos,0.1);
			}
			//ROS_INFO("Abs: %f Rel: %f   Counter: %f",ml.map(position+relative_zero,0,1023,-max_angle,max_angle),position,internal_counter);
			//std::cout<<"ROS\n";
			if(opening)
				ROS_INFO("Opening");
			else
				ROS_INFO("Stopped");
			ros::spinOnce();
			loop_rate.sleep();
	}
	return 0;
}
