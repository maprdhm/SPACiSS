/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/simulator.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pedsim_utils/geometry.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace pedsim;

Simulator::Simulator(const ros::NodeHandle& node) : nh_(node)
{
  dynamic_reconfigure::Server<SimConfig>::CallbackType f;
  f = boost::bind(&Simulator::reconfigureCB, this, _1, _2);
  server_.setCallback(f);
}

Simulator::~Simulator()
{
  // shutdown service servers and publishers
  pub_obstacles_.shutdown();
  pub_attractions_.shutdown();
  pub_waypoints_.shutdown();
  pub_agent_states_.shutdown();
  pub_agent_groups_.shutdown();
  pub_robot_position_.shutdown();
  mover_pub.shutdown();

  srv_pause_simulation_.shutdown();
  srv_unpause_simulation_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation()
{
  int queue_size = 0;
  nh_.param<int>("default_queue_size", queue_size, 10);
  ROS_INFO_STREAM("Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0 ? "NOTE: This means the queues are of infinite size!" : ""));

  // setup ros publishers
  pub_obstacles_ = nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_attractions_ = nh_.advertise<pedsim_msgs::Zones>("simulated_attractions", queue_size);
  pub_waypoints_ = nh_.advertise<pedsim_msgs::Zones>("simulated_waypoints", queue_size);
  pub_agent_states_ = nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ = nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ = nh_.advertise<nav_msgs::Odometry>("robot_position", queue_size);
  mover_pub = nh_.advertise<geometry_msgs::Twist>("/pedbot/control/cmd_vel", queue_size);

  // services
  srv_pause_simulation_ = nh_.advertiseService("pause_simulation", &Simulator::onPauseSimulation, this);
  srv_unpause_simulation_ = nh_.advertiseService("unpause_simulation", &Simulator::onUnpauseSimulation, this);

  // setup TF listener and other pointers
  transform_listener_.reset(new tf::TransformListener());
  robot_ = nullptr;

  std::string ns;
  nh_.param<std::string>("ns", CONFIG.ns, "");

  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "")
  {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false)
  {
    ROS_ERROR_STREAM("Could not load the scene file, please check the paths and param "
                     "names : "
                     << scene_file_param);
    return false;
  }

  nh_.param<bool>("enable_groups", CONFIG.groups_enabled, true);
  nh_.param<double>("group_size_lambda",  CONFIG.group_size_lambda, 1.1);
  if (CONFIG.group_size_lambda <= 0)
  {
    ROS_ERROR_STREAM("Invalid group size lambda");
    return false;
  }

  nh_.param<double>("groups_couples_proportion",  CONFIG.groups_couples_proportion, 0.0);
  nh_.param<double>("groups_friends_proportion",  CONFIG.groups_friends_proportion, 1.0);
  nh_.param<double>("groups_families_proportion",  CONFIG.groups_families_proportion, 0.0);
  nh_.param<double>("groups_coworkers_proportion",  CONFIG.groups_coworkers_proportion, 0.0);
  if (abs(CONFIG.groups_couples_proportion+ CONFIG.groups_friends_proportion+CONFIG.groups_families_proportion+CONFIG.groups_coworkers_proportion - 1.0)>0.001)
  {
    ROS_ERROR_STREAM("Invalid groups proportions: sum should be 1.0");
    return false;
  }

  nh_.param<double>("max_robot_speed", CONFIG.max_robot_speed, 3);

  int op_mode = 1;
  nh_.param<int>("robot_mode", op_mode, 1);
  CONFIG.robot_mode = static_cast<RobotMode>(op_mode);

  double initialX = 5.0;
  nh_.param<double>("pose_initial_x", initialX, 5.0);
  last_robot_pose_.getOrigin().setX(initialX);

  double initialY = 5.0;
  nh_.param<double>("pose_initial_y", initialY, 0.0);
  last_robot_pose_.getOrigin().setY(initialY);

  double initialTheta = 0.0;
  nh_.param<double>("pose_initial_theta", initialTheta, 0.0);
  last_robot_orientation_ = angleToQuaternion(initialTheta);

  nh_.param<double>("update_rate", CONFIG.updateRate, 25.0);
  nh_.param<double>("simulation_factor", CONFIG.simulationFactor, 1.0);
  nh_.param<double>("probability_random_stop", CONFIG.probability_random_stop, 0.005);
  nh_.param<bool>("enable_distraction", CONFIG.distraction_enabled, false);

  nh_.param<double>("angle_frontal_collision_risk", CONFIG.angleFrontalCollisionRisk, 25.0);
  nh_.param<double>("danger_radius", CONFIG.dangerRadius, 0.45);
  nh_.param<double>("risk_radius", CONFIG.riskRadius, 1.4);
  nh_.param<double>("ttc_low", CONFIG.ttcLow, -1.0);
  nh_.param<double>("ttc_up", CONFIG.ttcUp, 5.0);
  nh_.param<double>("ttc_stop", CONFIG.ttcStop, 2.0);
  nh_.param<double>("hesitation_threshold", CONFIG.hesitationThreshold, 0.1);

  paused_ = false;

  spawn_timer_ = nh_.createTimer(ros::Duration(5.0), &Simulator::spawnCallback, this);


  // If get trajectories from file, load trajectories here
   bool read_from_file = false;
   nh_.param<bool>("read_from_file", read_from_file, false);
   if(read_from_file) {
       std::string trajectories_file;
       nh_.param<string>("trajectories_file", trajectories_file, "");
       if (trajectories_file == "")
       {
         ROS_ERROR_STREAM("Invalid trajectories file: " << trajectories_file);
         return false;
       }
       SCENE.getInstance().loadTrajectoriesFromFile(trajectories_file);
   }

  return true;
}

void Simulator::runSimulation()
{
    sleep(1); //wait for rviz
    ros::Rate r(CONFIG.updateRate);
    while (ros::ok())
    {
      if (SCENE.getTime() < 1.5*CONFIG.getTimeStepSize() && !robot_)
      {
        // setup the robot
        for (Agent* agent : SCENE.getAgents())
        {
          if (agent->getType() == Ped::Tagent::ROBOT){
             robot_ = agent;
             checkRobotParams();
          }
        }
      }

      if (!paused_){
        updateRobotPositionFromTF();
        SCENE.moveAllAgents();

        if(SCENE.getTime() > 1.5*CONFIG.getTimeStepSize()){
           publishAgents();
           publishGroups();
           publishRobotCmd();
           publishRobotPosition();

           publishObstacles();  // TODO - no need to do this all the time.
           publishAttractions();  // TODO - no need to do this all the time.
           publishWaypoints();  // TODO - no need to do this all the time.
        }
      }
      ros::spinOnce();
      r.sleep();
    }
}

//Check parameters coherence
void Simulator::checkRobotParams()
{
   if(fabs(last_robot_pose_.getOrigin().getX() - robot_->getx()) > 0.001 ||
      fabs(last_robot_pose_.getOrigin().getY() - robot_->gety()) > 0.001 ||
      fabs(quaternionToYawAngle(last_robot_orientation_) - atan2(robot_->getvy(), robot_->getvx())) > 0.001)
   {
       ROS_ERROR_STREAM("robot x (" <<robot_->getx()<< "), y(" << robot_->gety() <<
                        ") and theta (" << atan2(robot_->getvy(), robot_->getvx())<<
                        ") in xml file "<<
                        "and robot pose_initial_x ("<< last_robot_pose_.getOrigin().getX()<<
                        "), pose_initial_y ("<<last_robot_pose_.getOrigin().getY()<<
                        ") and pose_initial_theta ("<< quaternionToYawAngle(last_robot_orientation_)<<") must be the same");
      ros::shutdown();
   }
}


void Simulator::reconfigureCB(pedsim_simulator::PedsimSimulatorConfig& config, uint32_t level)
{
  CONFIG.updateRate = config.update_rate;
  CONFIG.simulationFactor = config.simulation_factor;

  // update force scaling factors
  CONFIG.setObstacleForce(config.force_obstacle);
  CONFIG.setObstacleSigma(config.sigma_obstacle);
  CONFIG.setSocialForce(config.force_social);
  CONFIG.setGroupGazeForce(config.force_group_gaze);
  CONFIG.setGroupCoherenceForce(config.force_group_coherence);
  CONFIG.setGroupRepulsionForce(config.force_group_repulsion);
  CONFIG.setRandomForce(config.force_random);
  CONFIG.setAlongWallForce(config.force_wall);

  // puase or unpause the simulation
  if (paused_ != config.paused)
  {
    paused_ = config.paused;
  }

  ROS_INFO_STREAM("Updated sim with live config: Rate=" << CONFIG.updateRate
                                                        << " incoming rate=" << config.update_rate);
}

bool Simulator::onPauseSimulation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  paused_ = true;
  return true;
}

bool Simulator::onUnpauseSimulation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  paused_ = false;
  return true;
}

void Simulator::spawnCallback(const ros::TimerEvent& event)
{
  ROS_DEBUG_STREAM("Spawning new agents.");

  for (const auto& sa : SCENE.getSpawnAreas())
  {
    AgentCluster* agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));

    for (const auto& wp_name : sa->waypoints)
    {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::updateRobotPositionFromTF()
{
  if (!robot_)
    return;

  if (CONFIG.robot_mode == RobotMode::TELEOPERATION || CONFIG.robot_mode == RobotMode::CONTROLLED)
  {
    robot_->setTeleop(true);

    // Get robot position via TF
    tf::StampedTransform tfTransform;
    try
    {
      transform_listener_->lookupTransform("odom", CONFIG.ns+"base_footprint", ros::Time(0), tfTransform);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from base_footprint to odom failed. Reason: " << e.what());
      return;
    }

    const double x = tfTransform.getOrigin().x();
    const double y = tfTransform.getOrigin().y();
    const double dx = x - last_robot_pose_.getOrigin().x(), dy = y - last_robot_pose_.getOrigin().y();
    const double dt = tfTransform.stamp_.toSec() - last_robot_pose_.stamp_.toSec();
    double vx = dx / dt, vy = dy / dt;

    if (!std::isfinite(vx))
      vx = 0;
    if (!std::isfinite(vy))
      vy = 0;

    robot_->setX(x);
    robot_->setY(y);
    robot_->setvx(vx);
    robot_->setvy(vy);

    last_robot_pose_ = tfTransform;
  }
}

void Simulator::publishRobotPosition()
{
  if (robot_ == nullptr)
    return;

  nav_msgs::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = CONFIG.ns+"/base_footprint";

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  if (hypot(robot_->getvx(), robot_->getvy()) < 0.05)
  {
    robot_location.pose.pose.orientation = last_robot_orientation_;
  }
  else
  {
    robot_location.pose.pose.orientation = poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
    last_robot_orientation_ = robot_location.pose.pose.orientation;
  }

  robot_location.twist.twist.linear.x = robot_->getvx();
  robot_location.twist.twist.linear.y = robot_->getvy();
  pub_robot_position_.publish(robot_location);
}

// Publish approximative cmd corresponding to the robot movement
// rviz subscribe to position for now instead of cmd (because of unexplained bug)
void Simulator::publishRobotCmd()
{
   if (robot_ == nullptr)
     return;
   if (CONFIG.robot_mode == RobotMode::SOCIAL_DRIVE)
   {
      geometry_msgs::Twist cmd;
      Ped::Tvector last_orient = Ped::Tvector::fromPolar(Ped::Tangle::fromRadian(quaternionToYawAngle(last_robot_orientation_)));
      cmd.linear.x = robot_->getVelocity().length();
      cmd.angular.z = last_orient.angleTo(robot_->getVelocity()).toRadian()*CONFIG.updateRate;
      mover_pub.publish(cmd);
   }
}

void Simulator::publishAgents()
{
  if (SCENE.getAgents().size() < 1)
  {
    return;
  }

  pedsim_msgs::AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents())
  {
    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER)
    {
      state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT)
    {
      continue;
    }

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    // Projected trajectory
    geometry_msgs::PoseArray array;
    state.futurePositions = array;

    all_status.agent_states.push_back(state);
  }

  pub_agent_states_.publish(all_status);
}

void Simulator::publishGroups()
{
  if (!CONFIG.groups_enabled)
  {
    ROS_DEBUG_STREAM("Groups are disabled, no group data published: flag=" << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1)
  {
    return;
  }

  pedsim_msgs::AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto& ped_group : SCENE.getGroups())
  {
    if (ped_group->memberCount() <= 1)
      continue;

    pedsim_msgs::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto& member : ped_group->getMembers())
    {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_.publish(sim_groups);
}

void Simulator::publishObstacles()
{
  pedsim_msgs::LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto& obstacle : SCENE.getObstacles())
  {
    pedsim_msgs::LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle->getax();
    line_obstacle.start.y = obstacle->getay();
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle->getbx();
    line_obstacle.end.y = obstacle->getby();
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_.publish(sim_obstacles);
}

void Simulator::publishAttractions()
{
  pedsim_msgs::Zones sim_attractions;
  sim_attractions.header = createMsgHeader();
  for (const auto& attraction : SCENE.getAttractions())
  {
    pedsim_msgs::Zone attract;
    attract.pose.position.x = attraction->getPosition().x;
    attract.pose.position.y = attraction->getPosition().y;
    attract.size.x = attraction->getSize().width();
    attract.size.y = attraction->getSize().height();
    sim_attractions.zones.push_back(attract);
  }
  pub_attractions_.publish(sim_attractions);
}

void Simulator::publishWaypoints()
{
  pedsim_msgs::Zones waypoints;
  waypoints.header = createMsgHeader();
   std::set<Ped::Twaypoint *> wps;

  for (const Agent* a : SCENE.getAgents())
  {
     Ped::Twaypoint *wp = a->getCurrentWaypoint();
     wps.insert(wp);
  }

  for (const auto& waypoint : wps)
  {
    pedsim_msgs::Zone wp;
    wp.pose.position.x = waypoint->getPosition().x;
    wp.pose.position.y = waypoint->getPosition().y;
    wp.size.x = waypoint->getRadius()*2;
    waypoints.zones.push_back(wp);
  }
  pub_waypoints_.publish(waypoints);
}

std::string Simulator::agentStateToActivity(const AgentStateMachine::AgentState& state) const
{
  std::string activity = "Unknown";
  switch (state)
  {
    case AgentStateMachine::AgentState::StateWalking:
      activity = pedsim_msgs::AgentState::TYPE_INDIVIDUAL_MOVING;
      break;
    case AgentStateMachine::AgentState::StateGroupWalking:
      activity = pedsim_msgs::AgentState::TYPE_GROUP_MOVING;
      break;
    case AgentStateMachine::AgentState::StateQueueing:
      activity = pedsim_msgs::AgentState::TYPE_WAITING_IN_QUEUE;
      break;
    case AgentStateMachine::AgentState::StateShopping:
      break;
    case AgentStateMachine::AgentState::StateNone:
      break;
    case AgentStateMachine::AgentState::StateWaiting:
      break;
  }
  return activity;
}

std_msgs::Header Simulator::createMsgHeader() const
{
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = "odom";
  return msg_header;
}
