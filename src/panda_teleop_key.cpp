#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_B 0x62
#define KEYCODE_SPACE 0x20

#define DELTA 0.05

class PandaTeleop
{
public:
  PandaTeleop();
  void key_loop();

private:
  ros::NodeHandle nh;
  ros::Publisher delta_pub;
  ros::Publisher abs_pub;

  geometry_msgs::PoseStamped move_positive_x;
  geometry_msgs::PoseStamped move_negative_x;
  geometry_msgs::PoseStamped move_positive_y;
  geometry_msgs::PoseStamped move_negative_y;
  geometry_msgs::PoseStamped move_positive_z;
  geometry_msgs::PoseStamped move_negative_z;
  geometry_msgs::PoseStamped home_position;
};

PandaTeleop::PandaTeleop()
{
  delta_pub = nh.advertise<geometry_msgs::PoseStamped>("dagger/delta_pose", 100);
  move_negative_x.pose.position.x = -DELTA;
  move_positive_x.pose.position.x = DELTA;
  move_negative_y.pose.position.y = -DELTA;
  move_positive_y.pose.position.y = DELTA;
  move_positive_z.pose.position.z = DELTA;
  move_negative_z.pose.position.z = -DELTA;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "panda_teleop_key");
  PandaTeleop panda_teleop;

  signal(SIGINT, quit);

  panda_teleop.key_loop();
  return 0;
}

void PandaTeleop::key_loop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move Panda.");

  for (;;)
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch (c)
    {
      case KEYCODE_L:
        delta_pub.publish(move_positive_y);
        break;
      case KEYCODE_R:
        delta_pub.publish(move_negative_y);
        break;
      case KEYCODE_U:
        delta_pub.publish(move_positive_x);
        break;
      case KEYCODE_D:
        delta_pub.publish(move_negative_x);
        break;
      case KEYCODE_B:
        delta_pub.publish(move_positive_z);
        break;
      case KEYCODE_SPACE:
        delta_pub.publish(move_negative_z);
    }
  }
}