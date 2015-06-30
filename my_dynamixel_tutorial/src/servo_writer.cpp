#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <dynamixel_msgs/JointState.h>
#include <cmath>

class ServoWriter
{
	ros::NodeHandle nh_;
	ros::Publisher position_pub_;
	//ros::Subscriber position_sub_;
	public:
			ServoWriter()
			{
				//position_sub_ = nh_.subscribe("/tilt_controller/state", 1,&ServoWriter::positionSND, this);
				position_pub_ = nh_.advertise <std_msgs::Float64>("/tilt_controller/command", 1);
			};

			~ServoWriter()
			{
				ROS_INFO("Exiting");
			}
		void positionSND(std_msgs::Float64 pos_msg, double duration){
			ros::Rate loop_rate(50);
			double time_msg=0;
			double time_start=(double)ros::Time::now().toSec();
			while (time_msg< time_start+duration) /* Send command for duration seconds*/
			{
				position_pub_.publish(pos_msg);
				time_msg=(double)ros::Time::now().toSec();
				ros::spinOnce();
				loop_rate.sleep();
			}//time loop
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_writer");
	ServoWriter sw;
	ros::Rate loop_rate(50);
	std_msgs::Float64 target_position;
	float limit=0.8;
	while(ros::ok())
	{
		do{
			std::cout<<"Servo position?: ";
			std::cin>>target_position.data;
			if(fabs(target_position.data)<limit)
			{
				sw.positionSND(target_position,1);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}while(fabs(target_position.data)<limit);
		ros::shutdown();
	}
	return 0;
}
