#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <dynamixel_msgs/JointState.h>

class ServoReader
{
	ros::NodeHandle nh_;
	ros::Publisher position_pub_;
	ros::Subscriber position_sub_;
	std_msgs::Float64 target_position;
	public:
			ServoReader()
			{
				// Subscrive to input video feed and publish output video feed
				position_sub_ = nh_.subscribe("/tilt_controller/state", 1,&ServoReader::positionRC, this);
				position_pub_ = nh_.advertise <std_msgs::Int16>("position", 1);
			};

			~ServoReader()
			{
				ROS_INFO("Exiting");
			}
		void positionRC(const dynamixel_msgs::JointStateConstPtr& msg)
		{
			float pos=msg->current_pos;
			ROS_INFO("%f",pos);
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_reader");
	ServoReader sr;
	ros::spin();
	return 0;
}
