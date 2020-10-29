#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Odometry.h>

double g_updateRate, g_simulationFactor;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
nav_msgs::Odometry g_currentOdometry;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex;
int robot_mode;

//ros::Publisher mover_pub;

/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
void updateLoop()
{
  ros::Rate rate(g_updateRate);
  const double dt = g_simulationFactor / g_updateRate;

  while (true)
  {
    // Get current pose
    double x = g_currentPose.getOrigin().x();
    double y = g_currentPose.getOrigin().y();
    double theta = tf::getYaw(g_currentPose.getRotation());

    if(robot_mode == 2)
    {
       x = g_currentOdometry.pose.pose.position.x;
       y  = g_currentOdometry.pose.pose.position.y;
       geometry_msgs::Quaternion q = g_currentOdometry.pose.pose.orientation;
       double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
       double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
       theta = std::atan2(siny_cosp, cosy_cosp);
    }
    else
    {
       // Get requested translational and rotational velocity
       double v, omega;
       {
         boost::mutex::scoped_lock lock(mutex);
         v = g_currentTwist.linear.x;
         omega = g_currentTwist.angular.z;
       }

       // Simulate robot movement
       x += cos(theta) * v * dt;
       y += sin(theta) * v * dt;
       theta += omega * dt;
   }

    // Update pose
    g_currentPose.getOrigin().setX(x);
    g_currentPose.getOrigin().setY(y);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

    // Broadcast transform
    g_transformBroadcaster->sendTransform(
        tf::StampedTransform(g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

    rate.sleep();
  }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist)
{
  boost::mutex::scoped_lock lock(mutex);
  g_currentTwist = *twist;
}

void onOdometryReceived(const nav_msgs::Odometry::ConstPtr& odo)
{
  boost::mutex::scoped_lock lock(mutex);
  g_currentOdometry = *odo;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulate_diff_drive_robot");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");

  ros::Subscriber odometrySubscriber, twistSubscriber;

  std::string ns = "";
  privateHandle.param<std::string>("ns", ns, "");

  // Process parameters
  privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
  privateHandle.param<std::string>("robot_frame", g_robotFrame, ns+"/base_footprint");

  privateHandle.param<double>("/"+ns+"/pedsim_simulator/simulation_factor", g_simulationFactor,
                              1.0);                                                  // set to e.g. 2.0 for 2x speed
  privateHandle.param<double>("/"+ns+"/pedsim_simulator/update_rate", g_updateRate, 25.0);  // in Hz

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;

  privateHandle.param<double>("pose_initial_x", initialX, 0.0);
  privateHandle.param<double>("pose_initial_y", initialY, 0.0);
  privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);
  privateHandle.param<int>("robot_mode", robot_mode, 1);

  g_currentPose.getOrigin().setX(initialX);
  g_currentPose.getOrigin().setY(initialY);
  g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster.reset(new tf::TransformBroadcaster());

 if(robot_mode == 2)
 {
   odometrySubscriber= nodeHandle.subscribe<nav_msgs::Odometry>("/"+ns+"/pedsim_simulator/robot_position", 3, onOdometryReceived);
 }
 else {
    //handle cmd_vel
    twistSubscriber = nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);
 }
  // Run
  boost::thread updateThread(updateLoop);
  ros::spin();
}
