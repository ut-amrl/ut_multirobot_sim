#include <gflags/gflags.h>
#include <stdexcept>
#include <sys/signal.h>
#include <termios.h>

#include "ros/ros.h"
#include "ut_multirobot_sim/HumanControlCommand.h"

using ut_multirobot_sim::HumanControlCommand;

DEFINE_string(ros_topic, "/human1/command", "Topic for HumanControlCommands");

static struct termios old, current;

void init_termios() {
  tcgetattr(0, &old);
  current = old;
  current.c_lflag &= ~ICANON;
  current.c_lflag &= ~ECHO;
  tcsetattr(0, TCSANOW, &current);
}

void reset_termios(void) {
  tcsetattr(0, TCSANOW, &old);
}

char getch(void) {
  char ch;
  init_termios();
  ch = getchar();
  reset_termios();
  return ch;
}

void end_program(int sig) {
  reset_termios();
  ros::shutdown();
  exit(0);
}

void UpdateHCC(HumanControlCommand& hcc, char wasd) {
  switch (wasd) {
    case 'w':
      hcc.translational_velocity.y += 1.0;
      break;
    case 'a':
      hcc.translational_velocity.x -= 1.0;
      break;
    case 's':
      hcc.translational_velocity.y -= 1.0;
      break;
    case 'd':
      hcc.translational_velocity.x += 1.0;
      break;
    case ' ':
      hcc.translational_velocity.x = 0.0;
      hcc.translational_velocity.y = 0.0;
      break;
    default:
      throw std::invalid_argument("unknown command");
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "human_controller");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<HumanControlCommand>(FLAGS_ros_topic, 1);

  signal(SIGINT, end_program);

  char c;
  HumanControlCommand hcc;
  while (ros::ok()) {
    c = getch();
    switch (c) {
      case 'w':
      case 'a':
      case 's':
      case 'd':
      case ' ':
        UpdateHCC(hcc, c);
        hcc.header.stamp = ros::Time::now();
        pub.publish(hcc);
    }
  }
}