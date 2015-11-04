#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopHand
{
public:
  TeleopHand();
  void keyLoop();

private:


  ros::NodeHandle nh_;
  int code_;
  ros::Publisher key_pub_;

};

TeleopHand::TeleopHand():
  code_(0)
{
  key_pub_ = nh_.advertise<std_msgs::Int8>("/hand", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopHand teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();

  return(0);
}


void TeleopHand::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Listening from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to open/close hand.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    code_=0;
//    ROS_INFO("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("Opening");
        code_=3;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_INFO("Closing");
        code_=1;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_INFO("Closing");
        code_=1;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_INFO("Opening");
        code_=3;
        dirty = true;
        break;
    }


    std_msgs::Int8 key;
    key.data=code_;
    if(dirty ==true)
    {
      key_pub_.publish(key);
      dirty=false;
    }
  }


  return;
}


