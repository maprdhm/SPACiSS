#include "ros/ros.h"
#include <ros/node_handle.h>

/**
 * This class defines a timer to shutdown all nodes after a certain time.
 */

void stop_all_nodes(const ros::TimerEvent&)
{
  ROS_INFO_STREAM("Shutdown all nodes");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timer");
  ros::NodeHandle n("~");

  double time = 100;
  n.param<double>("time", time, 100);
  ros::Timer timer = n.createTimer(ros::Duration(long(time)), stop_all_nodes);

  ros::spin();

  return 0;
}
