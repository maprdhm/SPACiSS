/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
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

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/force/force.h>
#include "pedsim_simulator/element/obstacle.h"
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
#include <pedsim_simulator/rng.h>

#include <pedsim_simulator/element/agentgroup.h>
#include <QSet>
#include <fstream>

const double Agent::VISION_ANGLE_PED = 220.0;
const double Agent::VISION_DISTANCE_PED_MAX = 10.0;
const double Agent::ATTENTION_ANGLE_PED = 90.0;
const double Agent::ATTENTION_DISTANCE_PED_MAX = 5.0;
const double Agent::ANGLE_MAX_PED = 3.14;
const double Agent::DISTANCE_PED_MIN = 1.5; // minimum perception and attention distance in all directions

const double Agent::VISION_ANGLE_AV = 359.9; // if 360, Tangle normalized to 0...
const double Agent::VISION_DISTANCE_AV = 10.0;
const double Agent::ATTENTION_ANGLE_AV = 90.0;
const double Agent::ATTENTION_DISTANCE_AV = 5;
const double Agent::ANGLE_MAX_AV = 0.25;

const double Agent::MAXIMUM_MARGIN = 0.30;

bool SFM = false;

Agent::Agent()
{
  // initialize
  Ped::Tagent::setType(Ped::Tagent::ADULT);
  forceSigmaObstacle = CONFIG.sigmaObstacle;

  // waypoints
  currentDestination = nullptr;
  waypointplanner = nullptr;
  purpose = UNKNOWN;
  // state machine
  stateMachine = new AgentStateMachine(this);
  // group
  group = nullptr;

  hasMoved = false;
  initializePedestrianValues();
  isRunning = false;
  isStopped = false;
  isSteppingBack = false;
  emergencyStop = false;
  perceiveAV = false;
  collideAV = false;
}

void Agent::initializePedestrianValues(){
   // body ellipse from "Buchmüller and Weidmann, ‘Parameters of pedestrians, pedestrian traffic and walking facilities’, ETH Zurich, 2006".
   uniform_real_distribution<> wDistribution(0.39, 0.515);
   uniform_real_distribution<> thDistribution(0.235, 0.325);
   setEllipseWidth(wDistribution(RNG()));
   setEllipseThickness(thDistribution(RNG()));

   setAngleMax(Ped::Tangle::fromRadian(ANGLE_MAX_PED));
   setVisionAngle(Ped::Tangle::fromDegree(VISION_ANGLE_PED));
   setVisionDistance(VISION_DISTANCE_PED_MAX);
   setAttentionAngle(Ped::Tangle::fromDegree(ATTENTION_ANGLE_PED));
   setAttentionDistance(ATTENTION_DISTANCE_PED_MAX);

   setForceFactorSocial(CONFIG.forceSocial);
   setForceFactorObstacle(CONFIG.forceObstacle);

   uniform_real_distribution<> dDistribution(0.0, 1.0);
   setDistraction(dDistribution(RNG()));

   angleFrontalCollisionRisk = CONFIG.angleFrontalCollisionRisk;
   riskRadius = CONFIG.riskRadius; // increase risk radius to make ped avoid longer
   dangerRadius = CONFIG.dangerRadius;
   ttcLow = CONFIG.ttcLow;
   ttcUp = CONFIG.ttcUp;
   ttcStop = CONFIG.ttcStop;
   hesitationThreshold = CONFIG.hesitationThreshold;
}

Agent::~Agent()
{
  // clean up
  foreach (Force* currentForce, forces)
  {
    delete currentForce;
  }
}


void Agent::computeForces()
{
  // update forces
  desiredforce = desiredForce();
  if (forceFactorSocial > 0)
     socialforce = socialForce();
  if (forceFactorObstacle > 0)
    obstacleforce = obstacleForce();
  myforce = myForce(desiredDirection);

  collideAV = false;
  for (auto neighbor : getNeighbors())
  {
     if (neighbor->id == id)
        continue;
     if (neighbor->getType() == ROBOT){
         processCarInformation(neighbor);

         // is agent colliding with AV ?
         Ped::Tvector diff = neighbor->p - p;
         double physicalDistance = diff.length() -
               (this->getRadius(v.angleTo(diff),0.0) + neighbor->getRadius(neighbor->getWalkingDirection().angleTo(-diff),0.0));
           if(physicalDistance <= 0.0){
              collideAV = true;
              v = physicalForce();
              ROS_INFO_STREAM(id << " COLLIDE");
           }
      }
   }

  if (!perceiveAV) {
     this->isRunning = false;
     this->isStopped = false;
     this->isSteppingBack = false;
  }

  if(type==ROBOT){
     this->disableForce("Random");
  }
}

/// Calculates the desired force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::desiredForce()
{
  Ped::Tvector force;
  if (!disabledForces.contains("Desired"))
    force = Tagent::desiredForce();

  // inform users
  emit desiredForceChanged(force.x, force.y);

  return force;
}

Ped::Tvector Agent::socialForce() const
{
  Ped::Tvector force;
  if (!disabledForces.contains("Social")){

     // define relative importance of position vs velocity vector
     // (set according to Moussaid-Helbing 2009)
     const double lambdaImportance = 2.0;

     // define speed interaction
     // (set according to Moussaid-Helbing 2009)
     double gamma = 0.35;

     // define angular interaction
     // (set according to Moussaid-Helbing 2009)
     double n = 2;

     // define speed interaction
     // (set according to Moussaid-Helbing 2009)
     double n_prime = 3;

     //QSet <const AgentGroup*> perceivedGroups;
     QList <const Agent*> agents = this->getNeighbors();
     if(SFM)
         agents = (QList <const Agent*>&)SCENE.getAgents();

     for (const Agent* other : agents)
     {
       // don't compute social force to yourself
       if (other->id == id)
         continue;

       // compute difference between both agents' positions
       Ped::Tvector diff = other->p - p;

       //Semble être la meilleure combinaison
       if(other->getType() == ROBOT){
          gamma = 0.2;
       }

       //      if(other->getType() == ROBOT) diff /= 5;

       // Distance considering personal spaces
       double distance = diff.length() - (this->getRadius(v.angleTo(diff)) +
                         other->getRadius(other->getWalkingDirection().angleTo(-diff)));

       if(distance <0)
          distance = 0.0;

       // Distance without personal spaces (only body shapes)
       double physicalDistance = diff.length() - (this->getRadius(v.angleTo(diff),0.0) +
                                                  other->getRadius(other->getWalkingDirection().angleTo(-diff),0.0));

       // Between group members: personal space not considered
        if (this->isInGroupWith(other)) {
           distance = min(physicalDistance, 0.0);
         }

       Ped::Tvector diffDirection = diff.normalized();

       // compute difference between both agents' velocity vectors
       // Note: the agent-other-order changed here
       Ped::Tvector velDiff = v - other->v;

       // compute interaction direction t_ij
       Ped::Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
       double interactionLength = interactionVector.length();
       Ped::Tvector interactionDirection = interactionVector / interactionLength;

       // compute angle theta (between interaction and position difference vector)
       Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

       // compute model parameter B = gamma * ||D||
       double B = gamma * interactionLength;

       double thetaRad = theta.toRadian();
       double forceVelocityAmount = -exp(-distance / B - (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
       double forceAngleAmount = -theta.sign() * exp(-distance / B - (n * B * thetaRad) * (n * B * thetaRad));

       Ped::Tvector forceVelocity = forceVelocityAmount * interactionDirection;
       Ped::Tvector forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();

       if(other->getType() == ROBOT){
          forceVelocity.scale(2);
          forceAngle.scale(2);
       }

       else{
          // If in the same group: social force reduced
          if (this->isInGroupWith(other)) {
             forceVelocity.scale(0.05);
             forceAngle.scale(0.05);
          }
          // If member of a group : ignore attention zone
          else if (this->isInGroup()){}
          else {
            forceVelocity.scale(0.1);
            // Si voisin dans champ d'attention: force + forte
            if (getType()!= ROBOT && giveAttentionTo(other)) {
               forceVelocity.scale(5);
               forceAngle.scale(2);
            }
          }
       }

       force += forceVelocity + forceAngle;
     }

     // Cas où 2 agents se touchent
    // from Helbing et al., ‘Simulating Dynamical Features of Escape Panic’, 2000.
    force += this->physicalForce();
  }

  // inform users
  emit socialForceChanged(force.x, force.y);

  return force;
}

Ped::Tvector Agent::physicalForce() const
{
   Ped::Tvector force;

   // define body force factor
   double k_body = 12.0;

   // define sliding friction force factor
   double k_slidding = 24.0;

   QList <const Agent*> agents = this->getNeighbors();

   for (const Agent* other : agents)
   {
     // don't compute social force to yourself
     if (other->id == id)
       continue;

      // compute difference between both agents' positions
      Ped::Tvector diff = other->p - p;
      Ped::Tvector diffDirection = diff.normalized();
      // compute difference between both agents' velocity vectors
      Ped::Tvector velDiff = v - other->v;

      // Distance without personal spaces (only body shapes)
      double physicalDistance = diff.length() - (this->getRadius(v.angleTo(diff),0.0) +
                                                 other->getRadius(other->getWalkingDirection().angleTo(-diff),0.0));
      if(physicalDistance <= 0)
      {
         if(abs(physicalDistance) <= 0.001)
            physicalDistance = 0.1;

         double kbd = k_body;
         double ksl = k_slidding;

         if(other->getType() == ROBOT){
            kbd = k_body*500;
            ksl = k_slidding*1000;
         }
         // body force
         force += kbd * -physicalDistance * -diffDirection;

         // sliding friction force
          force -= ksl * -physicalDistance
                * Ped::Tvector::dotProduct(velDiff, (-diffDirection).leftNormalVector())
                * (-diffDirection).leftNormalVector();
      }
   }
   return force;
}

Ped::Tvector Agent::obstacleForce() const
{
/*
   Ped::Tvector force;
   if (!disabledForces.contains("Obstacle"))
      force = Tagent::obstacleForce();
   // inform users
   emit obstacleForceChanged(force.x, force.y);
   return force;
*/

  Ped::Tvector force;
  QList<const Obstacle*> obstacles = this->perceivedObstacles;

  if (!disabledForces.contains("Obstacle"))
  {
     if(obstacles.size()!=0)
     {
        // obstacle which is closest only
        Ped::Tvector minDiff, maxDiff;
        const Obstacle* minObstacle;
        double minDistance = INFINITY;

        const double k_body = 10.0;
        const double k_slidding = 20.0;

        foreach (const Obstacle* obstacle, obstacles)
        {
          Ped::Tvector closestPoint = obstacle->closestPoint(p);
          Ped::Tvector diff = closestPoint - p;
          Ped::Tangle angleDirectionObstacle = v.angleTo(diff);
           //obstacle thickness /2 to have distance between obstacle point and border
          double distance = diff.length()
                - this->getRadius(angleDirectionObstacle)
                - Obstacle::getObstacleThickness()/2;

          if (distance < minDistance)
          {
            minObstacle = obstacle;
            minDiff = diff;
            minDistance = distance;
            maxDiff = obstacle->farthestPoint(p) - p;
          }
        }

         // scale force
         double forceAmount = exp(-minDistance/(forceSigmaObstacle*4.0));

         // repulsion force
         force = forceAmount/5.0 * -minDiff.normalized();

         // if obstacle in attention area: conturn needed.
         if(giveAttentionTo(minObstacle))
         {
            //Go to the side closest to the destination
            Ped::Tangle farthest = desiredDirection.angleTo(maxDiff);
            Ped::Tangle closest = desiredDirection.angleTo(minDiff);

            Ped::Tvector forceDirection = (abs(closest.toDegree()) >= abs(farthest.toDegree())) ? maxDiff-minDiff : minDiff-maxDiff;


            /*
             * Follow the initial orientation
            Ped::Tvector obstacleDirection = minObstacle->getEndPoint() - minObstacle->getStartPoint();
            bool projectionPositive = (Ped::Tvector::dotProduct(v, obstacleDirection) >= 0);

            Ped::Tvector forceDirection = (projectionPositive) ? obstacleDirection : -obstacleDirection;
            */

            // Random
           /* Ped::Tvector obstacleDirection = minObstacle->getEndPoint() - minObstacle->getStartPoint();
            bool projectionPositive = (desiredDirection.angleTo(v).toDegree() < 0);

            Ped::Tvector forceDirection = (projectionPositive) ? obstacleDirection : -obstacleDirection;
            */

            forceDirection.normalize();
            force += forceDirection*forceAmount;
         }

         // When collide with a wall
        // from Helbing et al., ‘Simulating Dynamical Features of Escape Panic’, 2000.
        if(minDistance <= 0){
           // body force
           force += k_body * (-minDistance) * -minDiff.normalized();

           // sliding friction force
            force -= k_slidding * (-minDistance) * Ped::Tvector::dotProduct(v, (-minDiff).leftNormalVector())* (-minDiff).leftNormalVector();
        }
      }
   }
  // inform users
  emit obstacleForceChanged(force.x, force.y);

  return force;
}


Ped::Tvector Agent::myForce(Ped::Tvector desired) const
{
  // run additional forces
  Ped::Tvector forceValue;
  foreach (Force* force, forces)
  {
    // skip disabled forces
    if (disabledForces.contains(force->getName()))
    {
      // update graphical representation
      emit additionalForceChanged(force->getName(), 0, 0);
      continue;
    }

    // add force to the total force
    Ped::Tvector currentForce = force->getForce(desired);
    // → sanity checks
    if (!currentForce.isValid())
    {
      ROS_DEBUG("Invalid Force: %s", force->getName().toStdString().c_str());
      currentForce = Ped::Tvector();
    }
    forceValue += currentForce;

    // update graphical representation
    emit additionalForceChanged(force->getName(), currentForce.x, currentForce.y);
  }

  // inform users
  emit myForceChanged(forceValue.x, forceValue.y);

  return forceValue;
}

Ped::Twaypoint* Agent::getCurrentDestination() const
{
  return currentDestination;
}

Ped::Twaypoint* Agent::updateDestination()
{
  // assign new destination
  if (!destinations.isEmpty())
  {
    if (currentDestination != nullptr)
    {
      // cycle through destinations
      Waypoint* previousDestination = destinations.takeFirst();
      destinations.append(previousDestination);
    }
    currentDestination = destinations.first();
  }

  return currentDestination;
}

void Agent::updateState()
{
  // check state
  stateMachine->doStateTransition();
}

void Agent::move(double h)
{
   // do not move before robot initialization complete
   if(SCENE.getTime()<2*CONFIG.getTimeStepSize())
      return;

  if (getType() == Ped::Tagent::ROBOT)
  {
     if (CONFIG.robot_mode == RobotMode::TELEOPERATION || CONFIG.robot_mode == RobotMode::CONTROLLED)
     {
       // NOTE: Moving is now done by setting x, y position directly in
       // simulator.cpp
       // Robot's vx, vy will still be set for the social force model to work
       // properly wrt. other agents.

       // FIXME: This is a very hacky way of making the robot "move" (=update
       // position in hash tree) without actually moving it
       const double vx = getvx();
       const double vy = getvy();

       setvx(0);
       setvy(0);
       realisticMove(h);//Ped::Tagent::move(h);
       setvx(vx);
       setvy(vy);
     }
    else if (CONFIG.robot_mode == RobotMode::SOCIAL_DRIVE)
    {
      realisticMove(h);
    }
  }
  else if (getType() == Ped::Tagent::IMMOB)
   {
     setvx(0);
     setvy(0);
  }
  else if (getType() ==  Ped::Tagent::FROM_FILE)
  {
     moveToNextPositionFromFile();
  }
  else
  {
     realisticMove(h);
  }

  // inform users
  emit positionChanged(getx(), gety());
  emit velocityChanged(getvx(), getvy());
  emit accelerationChanged(getax(), getay());
}


void Agent::realisticMove(double stepSizeIn)
{
   // sum of all forces --> acceleration
   a = forceFactorDesired * desiredforce +
       forceFactorSocial * socialforce +
       forceFactorObstacle * obstacleforce +
       myforce;

   // realistic accelaration doesn't exceed 2g = 1.96 m/s² for walking pedestrians
   // Acceleration also used for AV for now (to be smooth) but in reality can brake more
   if(a.length()>1.96){
      a = a.normalized().scaled(1.96);
   }

   if(getType() == Agent::ROBOT)
      a = carAcceleration(a);

   // calculate the new velocity
   if (getTeleop() == false)
   {
     v = v + stepSizeIn * a;
   }

   // don't exceed maximal speed in normal walking condition
   double speed = v.length();
   if (!isRunning && speed > vmax){
        v = v.normalized() * vmax;
     }

   // If in danger, agents can run/sprint
   else if (isRunning && speed > runVmax){
        v = v.normalized() * runVmax;
     }

   // v must never be 0 to know agent orientation
   if(v.length()<0.0001){
      v=v.normalized()*0.0001;
   }

   // internal position update = actual move
   p += stepSizeIn * v;

   // notice scene of movement
   SCENE.getInstance().moveAgent(this);
}


Ped::Tvector Agent::carAcceleration(Ped::Tvector a)
{
   //AV turn on itself
   //   a.x = v.y;
   //   a.y = -v.x;

   // AV avoid walls only
   a = forceFactorDesired * desiredforce + forceFactorObstacle * obstacleforce;


   // Limit AV rotation angle (use sinus theorem)
   double angleMax = min(v.length()/4.0, this->getAngleMax().toRadian());

   // Too high rotation angle to be realistic
    if (v.angleTo(v+a).toRadian() > angleMax)
    {
       //AV very slow: does not slow down more, turn max
       if(v.length()<1)
       {
          double diffa = v.angleTo(v+a).toRadian() - angleMax;
          double rotation = -(diffa-asin(sin(angleMax)*(v.length()/a.length()))+asin(sin(v.angleTo(v+a).toRadian())*(v.length()/a.length())));
          double x = a.x * cos(rotation) - a.y * sin(rotation);
          double y = a.x * sin(rotation) + a.y * cos(rotation);
          a.x = x;
          a.y = y;
       }
       // slow down and turn max
       else
       {
          a = a.normalized().scaled((v.length()*sin(angleMax))/(sin(M_PI-(a).angleTo(-v).toRadian()- angleMax)));
       }
    }
    // Too high rotation angle to be realistic
    else if (v.angleTo(v+a).toRadian()< - angleMax)
    {
       //AV very slow: does not slow down more, turn max
       if(v.length()<1)
       {
          double diffa = -v.angleTo(v+a).toRadian() - angleMax;
          double rotation = diffa-asin(sin(angleMax)*(v.length()/a.length()))+asin(sin(-v.angleTo(v+a).toRadian())*(v.length()/a.length()));
          double x = a.x * cos(rotation) - a.y * sin(rotation);
          double y = a.x * sin(rotation) + a.y * cos(rotation);
          a.x = x;
          a.y = y;
       }
       // slow down and turn max
       else
       {
         a = a.normalized().scaled((v.length()*sin(angleMax))/(sin(M_PI-(-v).angleTo(a).toRadian()-angleMax)));
      }
    }

    if(emergencyStop == true){
       a = -v;
    }
    return a;
}

Ped::Tangle Agent::getAngleMax(){
   return angleMax;
}

void Agent::setAngleMax(Ped::Tangle angle)
{
   this->angleMax = angle;
}

Ped::Tangle Agent::getVisionAngle() const{
   return visionAngle;
}

double Agent::getVisionAngleDeg() const{
   return visionAngle.toDegree(Ped::Tangle::AngleRange::PositiveOnlyRange);
}

void Agent::setVisionAngle(Ped::Tangle visionAngleIn){
   visionAngle=visionAngleIn;
}

Ped::Tangle Agent::getAttentionAngle() const{
   return attentionAngle;
}
double Agent::getAttentionAngleDeg() const{
   return attentionAngle.toDegree(Ped::Tangle::AngleRange::PositiveOnlyRange);
}
void Agent::setAttentionAngle(Ped::Tangle attentionAngleIn){
   attentionAngle= attentionAngleIn;
}
double Agent::getVisionDistance() const{
   return visionDistance;
}
void Agent::setVisionDistance(double visionDistanceIn){
   visionDistance = visionDistanceIn;
}
double Agent::getAttentionDistance() const{
   return attentionDistance;
}
void Agent::setAttentionDistance(double attentionDistanceIn){
   attentionDistance= attentionDistanceIn;
}


void Agent::moveToNextPositionFromFile()
{
   std::map<int, std::map<int, Ped::Tvector>> realTrajectories = SCENE.getInstance().getRealtrajectories();
   int numFrame = static_cast<int>(SCENE.getTime() * CONFIG.updateRate);

   if(realTrajectories.count(numFrame) > 0 &&
     realTrajectories.at(numFrame).count(id+1) > 0)
   {
      double newX = realTrajectories.at(numFrame).at(id+1).x;
      double newY = realTrajectories.at(numFrame).at(id+1).y;
      v.x = (newX - p.x) / CONFIG.getTimeStepSize();
      v.y = (newY - p.y) / CONFIG.getTimeStepSize();
      p.x = newX;
      p.y = newY;
      hasMoved = true;
   }
   // if already moved and do not move more, move to 1000 and remove then
   else if (hasMoved){
      p.x = 1000.0;
      p.y = 1000.0;
   }
}

const QList<Waypoint*>& Agent::getWaypoints() const
{
  return destinations;
}

bool Agent::setWaypoints(const QList<Waypoint*>& waypointsIn)
{
  destinations = waypointsIn;
  return true;
}

bool Agent::addWaypoint(Waypoint* waypointIn)
{
  destinations.append(waypointIn);
  return true;
}

bool Agent::removeWaypoint(Waypoint* waypointIn)
{
  const int removeCount = destinations.removeAll(waypointIn);

  return (removeCount > 0);
}

bool Agent::needNewDestination() const
{
  if (waypointplanner == nullptr)
    return (!destinations.isEmpty());
  else
  {
    // ask waypoint planner
    return waypointplanner->hasCompletedDestination();
  }
}

Ped::Twaypoint* Agent::getCurrentWaypoint() const
{
  // sanity checks
  if (waypointplanner == nullptr)
    return nullptr;

  // ask waypoint planner
  return waypointplanner->getCurrentWaypoint();
}

bool Agent::isInGroup() const
{
  return (group != nullptr);
}

bool Agent::isInGroupWith(const Agent* other) const
{
    if(this->isInGroup() && this->getGroup()->getMembers().contains((Agent*)other))
       return true;
    return false;
}

AgentGroup* Agent::getGroup() const
{
  return group;
}

void Agent::setGroup(AgentGroup* groupIn)
{
  group = groupIn;
}

bool Agent::addForce(Force* forceIn)
{
  forces.append(forceIn);

  // inform users
  emit forceAdded(forceIn->getName());

  // report success
  return true;
}

bool Agent::removeForce(Force* forceIn)
{
  int removeCount = forces.removeAll(forceIn);

  // inform users
  emit forceRemoved(forceIn->getName());

  // report success if a Behavior has been removed
  return (removeCount >= 1);
}

AgentStateMachine* Agent::getStateMachine() const
{
  return stateMachine;
}

WaypointPlanner* Agent::getWaypointPlanner() const
{
  return waypointplanner;
}

void Agent::setWaypointPlanner(WaypointPlanner* plannerIn)
{
  waypointplanner = plannerIn;
}

double Agent::getEllipseWidth() {
  return this->ellipseWidth;
}
void Agent::setEllipseWidth(double width){
  this->ellipseWidth=width;
}

double Agent::getEllipseThickness(){
  return this->ellipseThickness;
}
void Agent::setEllipseThickness(double thickness){
  this->ellipseThickness=thickness;
}

QList<const Agent*> Agent::getNeighbors() const
{
  // upcast neighbors
  QList<const Agent*> output;
  for (const Ped::Tagent* neighbor : neighbors)
  {
    const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
    if (upNeighbor != nullptr)
      output.append(upNeighbor);
  }
  return output;
}

void Agent::disableForce(const QString& forceNameIn)
{
  // disable force by adding it to the list of disabled forces
  disabledForces.append(forceNameIn);
}

void Agent::enableAllForces()
{
  // remove all forces from disabled list
  disabledForces.clear();
}

void Agent::setPosition(double xIn, double yIn)
{
  // call super class' method
  Ped::Tagent::setPosition(xIn, yIn);

  // inform users
  emit positionChanged(xIn, yIn);
}

void Agent::setX(double xIn)
{
  setPosition(xIn, gety());
}

void Agent::setY(double yIn)
{
  setPosition(getx(), yIn);
}

// When set max walking speed, set max running speed
void Agent::setVmax(double vmax)
{
  this->vmax = vmax;
  // Running speed = [2-3] * walking speed
  // from paper "Analysis of Car-Pedestrian Impact Scenarios for the Evaluation of a Pedestrian Sensor System Based on Accident Data from Sweden"
  uniform_real_distribution<> distribution(2, 3);
  setRunVmax(vmax * distribution(RNG()));
}

void Agent::setRunVmax(double runVmax)
{
  this->runVmax = runVmax;
}

void Agent::setType(Ped::Tagent::AgentType typeIn)
{
  // call super class' method
  Ped::Tagent::setType(typeIn);

  if(typeIn == ROBOT){
     this->setVmax(CONFIG.max_robot_speed);
     // Ellipse englobante
     this->setEllipseThickness(4.6);
     this->setEllipseWidth(2.4);
     this->SetRadius(2.3);
     /* Av size from data CITR*/
    /* this->setEllipseThickness(2.2);
     this->setEllipseWidth(1.2);
     this->SetRadius(1.1);*/
     //Best angle to fit with given turning circle
     this->setAngleMax(Ped::Tangle::fromRadian(ANGLE_MAX_AV));
     this->setVisionAngle(Ped::Tangle::fromDegree(VISION_ANGLE_AV));
     this->setVisionDistance(VISION_DISTANCE_AV);
     this->setAttentionAngle(Ped::Tangle::fromDegree(ATTENTION_ANGLE_AV));
     this->setAttentionDistance(ATTENTION_DISTANCE_AV);
     this->setForceFactorSocial(CONFIG.forceSocial *10);
     this->setForceFactorObstacle(CONFIG.forceObstacle*5);
     this->setDistraction(0.0);
  }
  else{
     normal_distribution<double> speed(1.34, 0.26);
     this->setVmax(speed(RNG()));
     this->SetRadius(0.35);
     initializePedestrianValues();
     if (typeIn == ELDER)
     {
       // Old people slow!
       this->setVmax(1.0);
       this->setForceFactorDesired(0.5);
     }
  }
  // inform users
  emit typeChanged(typeIn);
}

Agent::AgentPurpose Agent::getPurpose() const
{
   return purpose;
}

void Agent::setPurpose(AgentPurpose purposeIn)
{
   this->purpose = purposeIn;

   if(this->type != ROBOT && this->type != ELDER)
   {
      normal_distribution<double> distribution(1.34, 0.26);
      normal_distribution<double> distributionWork(1.55, 0.26);
      normal_distribution<double> distributionLeisure(1.13, 0.26);
      switch (purposeIn)
      {
          case WORK:
             this->setVmax(distributionWork(RNG()));
             break;
          case LEISURE:
             this->setVmax(distributionLeisure(RNG()));
             break;
          default:
             this->setVmax(distribution(RNG()));
      }
   }

   // inform users
   emit purposeChanged(purposeIn);
}

bool Agent::isPerceivingAV() const{
   return perceiveAV;
}

string Agent::getDecision() const{
   if (isStopped)
      return "stop";
   else if (isRunning)
      return "run";
   else if (isSteppingBack)
      return "back";
   else return "-";

}

bool Agent::isCollidingAV() const{
   return collideAV;
}



double Agent::getDistraction() const{
   return distraction;
}

void Agent::setDistraction(double distractionIn){
    if(type == ROBOT)
       return;
   //If distraction disabled: force to 0 (not distracted at all)
   if (!CONFIG.distraction_enabled)
      distractionIn = 0.0;

   // While car is perceived: not distracted
   if(perceiveAV){
      distractionIn = 0.0;
   }
   double distractionLimited = max(min(distractionIn, 1.0), 0.0);
   this->distraction = distractionLimited;
   updateVision(distraction);
   updateAttention(distraction);
}

// Change pedestrian distraction value
void Agent::varyDistraction(){
   if(type == ROBOT)
      return;
   uniform_real_distribution<> dDistribution(0, 1);
   setDistraction(dDistribution(RNG()));
}

/*
 * Get the area perceived by the agent in m²
 * VISION_ANGLE/360 * PI * VISION_DISTANCE² + (360-VISION_ANGLE)/360 * PI *DISTANCE_PED_MIN²
*/
double Agent::getVisionArea() const
{
   double visionArea = getVisionAngleDeg()/360.0 * M_PI * visionDistance * visionDistance +
         (360.0 - getVisionAngleDeg())/360.0 * M_PI * DISTANCE_PED_MIN*DISTANCE_PED_MIN;
   return visionArea;
}

// Update the vision distance with distraction : -8.5x + 10 m
void Agent::updateVision(double distraction){
   double visionDistance = (DISTANCE_PED_MIN-VISION_DISTANCE_PED_MAX) * distraction + VISION_DISTANCE_PED_MAX;
   visionDistance = max(min(visionDistance, VISION_DISTANCE_PED_MAX), DISTANCE_PED_MIN);
   this->setVisionDistance(visionDistance);
}

// Update the attention distance with distraction: -3.5x + 5 m
void Agent::updateAttention(double distraction){
   double attentionDistance = (DISTANCE_PED_MIN-ATTENTION_DISTANCE_PED_MAX) * distraction + ATTENTION_DISTANCE_PED_MAX;
   attentionDistance = max(min(attentionDistance, ATTENTION_DISTANCE_PED_MAX), DISTANCE_PED_MIN);
   this->setAttentionDistance(attentionDistance);
}

Ped::Tvector Agent::getDesiredDirection() const
{
  return desiredforce;
}

Ped::Tvector Agent::getWalkingDirection() const
{
  return v;
}

Ped::Tvector Agent::getSocialForce() const
{
  return socialforce;
}

Ped::Tvector Agent::getObstacleForce() const
{
  return obstacleforce;
}

Ped::Tvector Agent::getMyForce() const
{
  return myforce;
}

QPointF Agent::getVisiblePosition() const
{
  return QPointF(getx(), gety());
}

void Agent::setVisiblePosition(const QPointF& positionIn)
{
  // check and apply new position
  if (positionIn != getVisiblePosition())
    setPosition(positionIn.x(), positionIn.y());
}

// Return ellipse radius according to rotation angle
double Agent::getRadius(Ped::Tangle angle) const
{
   double margin = getEllipseMargin();
   return getRadius(angle, margin);
}

// Return ellipse radius according to rotation angle and specific margin
double Agent::getRadius(Ped::Tangle angle, double margin) const
{
   double ellipseTh = ellipseThickness;
   if(abs(angle.toDegree())>90.0) {
      ellipseTh += margin;
   }
   else {
      ellipseTh += 4.0*margin;
   }

   double ellipseW = ellipseWidth + 2.0/3*margin;


   if (type==ROBOT){
       ellipseTh = ellipseThickness;
       if(abs(angle.toDegree())>90.0)
          ellipseTh += 2.0*margin; // back
       else
          ellipseTh += 2.0*margin; // front
       ellipseW = ellipseWidth + 2.0*margin; //side
   }

   double epsilon = (ellipseTh/2) * (ellipseW/2);
   return epsilon / (sqrt (pow(ellipseTh/2.0, 2)*pow(sin(angle.toRadian()), 2) +
                           pow(ellipseW/2.0, 2)*pow(cos(angle.toRadian()), 2)
                           ));
}

/*
 * Return margin between 0 and 0.30 according to perceived density
 * Return 0.30 if density <= 0.18p/m² and 0.0 if density >= 0.71p/m²
 */
double Agent::getEllipseMargin() const
{
   if(SFM) {
      return 0.0;
   }
   //Personal space for AV : 2m margin
   if(this->type == ROBOT) {
      return 2.0;
   }
   int nbNeighbors = this->neighbors.size();
   double density = 0.0;

   if (nbNeighbors != 0){
      density = nbNeighbors / getVisionArea();
   }

   double margin = min(max(-0.566 * density + 0.402, 0.0), MAXIMUM_MARGIN);

   return margin;
}

QString Agent::toString() const
{
  return tr("Agent %1 (@%2,%3)").arg(getId()).arg(getx()).arg(gety());
}

/**************** PERCEPTION ***************/
/*
 * Get the list of perceived agents
 */
QList<const Agent*> Agent::updatePerceivedNeighbors()
{
  QList<const Agent*> perceived;
  set<const Ped::Tagent*> neighborsSet;

  this->perceiveAV = false;
  double angle = getVisionAngleDeg();
  double visionDistance = getVisionDistance();

  for (const Ped::Tagent* neighbor : scene->getNeighbors(this->getx(), this->gety(), visionDistance))
  {
    const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
    if(upNeighbor == nullptr || upNeighbor->getId() == id)
        continue;

    if (perceiveAgent(upNeighbor, visionDistance, angle)){
        perceived.append(upNeighbor);
        neighborsSet.insert(upNeighbor);
        if (upNeighbor->getType() == ROBOT){
           // ROS_INFO_STREAM("PEDESTRIAN ID " << getId() << "PERCEIVES A CAR ID" << neighbor->getId());
            this->perceiveAV = true;
        }
    }
  }
  this->neighbors = neighborsSet;
  return perceived;
}

/*
 * Get the list of perceived obstacles
 */
void Agent::updatePerceivedObstacles()
{
   this->perceivedObstacles = getPerceivedObstacles();
}

/*
 * Return true if current agent perceive the perceived agent.
 */
bool Agent::perceiveAgent(const Agent* perceived, double distanceMax, double angleViewDegree) const
{
   Ped::Tvector neighborDirection = perceived->getPosition() - p;

   double physicalDistance = neighborDirection.length() - (this->getRadius(v.angleTo(neighborDirection),0.0) +
                                              perceived->getRadius(perceived->getWalkingDirection().angleTo(-neighborDirection),0.0));

   Ped::Tangle angle = this->getWalkingDirection().angleTo(neighborDirection);

   //The neighbor is perceived if it is at maximum distanceMax meters and in the zone areaViewDegree OR
   // if it is at maximum DISTANCE_PED_MIN meters OR
   // if it is in the same group OR
   // if it is a car and it is at maximum distanceMax/2 meters (whatever the degree is)
    if ((physicalDistance <= distanceMax && abs(angle.toDegree()) <= angleViewDegree/2.0)
        || physicalDistance <= DISTANCE_PED_MIN || this->isInGroupWith(perceived)
        || (perceived->getType()==ROBOT && physicalDistance <= distanceMax/3)) // Calibrated from data
    {
        return true;
    }
    return false;
}


void Agent::processCarInformation(const Agent* car)
{
   // Physical collision radius
   double collisionRadius =  this->agentRadius + car->agentRadius;
   // Danger radius
   double radiusDanger = this->agentRadius + car->agentRadius + dangerRadius;
   // Risk radius
   double radiusRisk =  this->agentRadius + car->agentRadius + riskRadius;

   // AV speed
   double carvx = car->getvx();
   double carvy = car->getvy();

   double carx = car->getPosition().x;
   double cary = car->getPosition().y;

   // Velocity vector of pedestrian with his ideal speed
   Ped::Tvector pedIdealVelocity = this->getVelocity().normalized().scaled(vmax);
  if(isSteppingBack)
     pedIdealVelocity = this->desiredDirection.normalized().scaled(vmax/3);

   double b = 2*(this->getx()-carx)*(pedIdealVelocity.x-carvx) + 2*(this->gety()-cary)*(pedIdealVelocity.y-carvy);
   double a = pow(pedIdealVelocity.x-carvx, 2)+ pow(pedIdealVelocity.y-carvy,2);
   double cDanger = pow(this->getx()-carx, 2) + pow(this->gety()-cary,2)-pow(radiusDanger,2);
   double cRisk = pow(this->getx()-carx, 2) + pow(this->gety()-cary,2)-pow(radiusRisk,2);
   double cCollision = pow(this->getx()-carx, 2) + pow(this->gety()-cary,2)-pow(collisionRadius,2);
   double deltaDanger = pow(b, 2)-4*a*cDanger;
   double deltaRisk = pow(b, 2)-4*a*cRisk;
   double deltaCollision = pow(b, 2)-4*a*cCollision;
   double ttc = 10;


   // if physical collision possible (in past or present or future)
   if(deltaDanger>=0)
   {
      // first time to collision
     ttc = min((-b+sqrt(deltaDanger))/(2*a), (-b-sqrt(deltaDanger))/(2*a));

      // Colision in near future (< 5s)
      if(ttc>ttcLow && ttc<ttcUp)
      {
         Ped::Tvector pedPos = p;
         Ped::Tvector pedVelo = pedIdealVelocity;
         if(isInGroup()) {
            pedVelo = group->getWalkingDirection();
            if(deltaCollision>0 && min((-b+sqrt(deltaCollision))/(2*a),(-b-sqrt(deltaCollision))/(2*a))<ttcStop ){
               myforce = Ped::Tvector();
               //ROS_INFO_STREAM("temp leave group");
            } else {
              pedPos = group->getCenterOfMass();
            }
         }


         Ped::Tangle diffAngle = Ped::Tvector(carvx, carvy).angleTo(pedVelo);
         double degreeDiffAngle = diffAngle.toDegree();
         //ROS_INFO_STREAM(id<< " "<<"ANGLE DIFFERENCE CAR - PEDESTRIAN IN DEGREE " << degreeDiffAngle);

         // if car in front or back -> turn instead of following social force
         // it is not enough to set social force stronger, because may cause to follow the car move...
         if (!isSteppingBack && (degreeDiffAngle > (180.0 - angleFrontalCollisionRisk) ||  degreeDiffAngle < angleFrontalCollisionRisk - 180.0
             || (degreeDiffAngle >0 && degreeDiffAngle<angleFrontalCollisionRisk) || (degreeDiffAngle<0 && degreeDiffAngle > -angleFrontalCollisionRisk))){

            this->isStopped = false;
            this->isRunning = false;

            Ped::Tvector diffDirection = (pedPos-car->p).normalized();
            Ped::Tangle angle = car->v.angleTo(diffDirection);
            Ped::Tvector turnVector = angle.sign() * car->v.leftNormalVector().normalized();
            socialforce = turnVector + physicalForce();
           // ROS_INFO_STREAM(id<<"tttuuuurn");
         }
         else{
            // Bearing angle considers the AV size now + 2m margin!!
            Ped::Tvector carNearestSide = Ped::Tvector(car->p.x + (car->getRadius((Ped::Tvector(carvx, carvy).angleTo(pedPos-car->p)),2.0) * (pedPos-car->p)).x/*(car->getVelocity().normalized().x)*/,
                                  car->p.y + (car->getRadius((Ped::Tvector(carvx, carvy).angleTo(pedPos-car->p)),2.0) * (pedPos-car->p)).y);
            // Bearing angle from ped point of view
            Ped::Tangle bearingAngle = pedVelo.angleTo(carNearestSide - pedPos);
            double bearingAngleDeriv = (carNearestSide - pedPos).angleTo((carNearestSide - pedPos)+(Ped::Tvector(carvx, carvy) - pedVelo)).toRadian();
            // Bearing angle from car point of view
            Ped::Tangle bearingAngleC = (Ped::Tvector(carvx, carvy).angleTo(pedPos-carNearestSide));
            double bearingAngleDerivC = (pedPos-carNearestSide).angleTo((pedPos-carNearestSide)+(pedVelo-Ped::Tvector(carvx, carvy))).toRadian();

            double cx = ((this->getx()+pedIdealVelocity.x*ttc)*this->agentRadius + (car->getx()+carvx*ttc)*car->agentRadius)/radiusDanger;
            double cy = ((this->gety()+pedIdealVelocity.y*ttc)*this->agentRadius + (car->gety()+carvy*ttc)*car->agentRadius)/radiusDanger;
            Ped::Tvector collisionPoint = Ped::Tvector(cx, cy);
            Ped::Tvector pedToColl = collisionPoint - Ped::Tvector(this->getx()+pedIdealVelocity.x * ttc,this->gety()+pedIdealVelocity.y * ttc);

            // if both converge/diverge -> already crossed
            if((bearingAngle.sign()*bearingAngleDeriv<0 && bearingAngleC.sign()*bearingAngleDerivC<0)
                  || (bearingAngle.sign()*bearingAngleDeriv>0 && bearingAngleC.sign()*bearingAngleDerivC>0)
                  ||abs(pedToColl.angleTo(this->getWalkingDirection()).toDegree())>90){
               //ROS_INFO_STREAM("already crossed");
               this->isStopped = false;
               this->isSteppingBack=false;
               this->isRunning=false;
            }

           else{
            //ROS_INFO_STREAM(id<<" "<<bearingAngle.sign()*bearingAngleDeriv<<" "<<bearingAngleC.sign()*bearingAngleDerivC);
            if(!isSteppingBack &&!isRunning && !isStopped && fabs(bearingAngleDeriv) <= hesitationThreshold)
            {
              // ROS_INFO_STREAM("hesitation: run or stop");

               // Follow group decision
               if(this->isInGroup() && pedPos == group->getCenterOfMass())
               {
                  AgentGroup* group = getGroup();

                  // check all group members
                  foreach (Agent* member, group->getMembers())
                  {
                     // ignore agent himself
                     if (member == this)
                       continue;
                     if(member->isStopped){
                        this->isSteppingBack=true;//this->wantStop();
                        break;
                     }
                     else if(member->isRunning){
                        this->wantRun();
                        break;
                     }
                 }
               }
               // If no decision (not in groupe or first to decide)
               if(!isRunning && !isStopped){
                  // prudent
                  if(ELDER)
                     this->wantStop();
                  else {
                     //normal
                     int a = rand()%2;
                     if(a == 1){
                        this->wantStop();
                     }
                     else{
                          this->wantRun();
                     }
                  }
               }
            }


            // if ped will arrive first
            else if (bearingAngle.sign()*bearingAngleDeriv > hesitationThreshold || (bearingAngle.sign()*bearingAngleDeriv >=0.0 && isRunning)){
               this->wantRun();
            }
            // if ped will arrive second
            else if (bearingAngle.sign()*bearingAngleDeriv < -hesitationThreshold || (bearingAngle.sign()*bearingAngleDeriv <=0.0 && isStopped)){
               if (bearingAngle.sign()*bearingAngleDeriv > -hesitationThreshold && isStopped ){
                  this->isSteppingBack=true;
               }
               else{
                  this->wantStop();
                  this->isSteppingBack=false;
               }
            }
          }
      }
       }
      else if (isSteppingBack){
        this->isSteppingBack = false;
        this->wantStop();
     }
   }
    else if (isSteppingBack){
      this->isSteppingBack = false;
      this->wantStop();
   }


   // if will be in risky distance
   if(deltaRisk >= 0)
   {
      // last time of risk
      double ttcRisk = max((-b+sqrt(deltaRisk)/2*a), (-b-sqrt(deltaRisk)/2*a));
      // if risky not ended and running/stopped, continue to run/stop
      if (ttcRisk>0){
        // myforce = Ped::Tvector(); //ignore group forces
         if (isRunning || isStopped || isSteppingBack){}
      }
      else {
         this->isRunning = false;
         this->isStopped = false;
         this->isSteppingBack = false;
      }
   }
   else {  //out of risky zone
      this->isRunning = false;
      this->isStopped = false;
      this->isSteppingBack = false;
   }

   // action: run or stop
   if (isRunning){
      socialforce = physicalForce();
      if(isInGroup())
         desiredforce = (group->getWalkingDirection().normalized().scaled(runVmax) - v)/relaxationTime;
      else desiredforce = (v.normalized().scaled(runVmax) - v)/relaxationTime;
   }
   else if (isSteppingBack)
   {
      socialforce = - car->v.normalized() + physicalForce();
      desiredforce = -desiredforce;
   }
   else if (isStopped){
     socialforce = - car->v.normalized() + physicalForce();
     if(ttc<ttcStop){
         desiredforce = (-v/relaxationTime);
     }
   }
}

/*
 * Agent wants to stop
 */
void Agent::wantStop(){
   this->isRunning = false;
   this->isStopped = true;
}

/*
 * Agent wants to run
 */
void Agent::wantRun(){
   this->isRunning = true;
   this->isStopped = false;
}

/*
 * Return true if current agent must give attention to neighbor
 */
bool Agent::giveAttentionTo(const Agent* neighbor) const
{
   if (perceiveAV)
      return false;

    if (this->getNeighbors().contains(neighbor))
    {
      Ped::Tvector neighborDirection = neighbor->p - p;
      double physicalDistance = neighborDirection.length() - (this->getRadius(v.angleTo(neighborDirection),0.0) +
                                                 neighbor->getRadius(neighbor->getWalkingDirection().angleTo(-neighborDirection),0.0));
      Ped::Tangle angle = v.angleTo(neighborDirection);

       if (abs(angle.toDegree()) <= getAttentionAngleDeg()/2.0 && physicalDistance <= getAttentionDistance()
           || physicalDistance <= DISTANCE_PED_MIN)
       {
         return true;
       }
    }
    return false;
}

/*
 * Return true if current agent must give attention to group
 */
bool Agent::giveAttentionTo(const AgentGroup* group, double distance, Ped::Tangle angle) const
{
     bool attention = false;
      for (const Agent* a : group->getMembers())
      {
         if(giveAttentionTo(a))
            attention = true;
      }

      if ( attention ||
          abs(angle.toDegree()) <= getAttentionAngleDeg()/2.0 && distance <= getAttentionDistance() ||
          distance <= DISTANCE_PED_MIN)
      {
         return true;
      }

    return false;
}

/*
 * Return the perceived obstacles
 */
QList<const Obstacle*> Agent::getPerceivedObstacles() const
{
   QList<const Obstacle*> perceivedObstacles;

   for (const Obstacle* obstacle : SCENE.getObstacles())
   {
       Ped::Tvector closestPoint = obstacle->closestPoint(p);
       Ped::Tvector diff = closestPoint - p;

       double distance = diff.length()- this->getRadius(v.angleTo(diff),0.0)- Obstacle::getObstacleThickness()/2;

      Ped::Tangle angle = this->getWalkingDirection().angleTo(diff);
      if (distance <= getVisionDistance() && abs(angle.toDegree()) <= getVisionAngleDeg()/2.0
          || distance <= DISTANCE_PED_MIN)
      {
            perceivedObstacles.append(obstacle);
      }
   }
   return perceivedObstacles;
}


/*
 * Return the perceived attractions
 */
QList<AttractionArea*> Agent::getPerceivedAttractions() const
{
   QList<AttractionArea*> perceivedAttractions;

   for (AttractionArea* attraction : SCENE.getAttractions())
   {
      Ped::Tvector diff = attraction->getPosition() - p;
      double distance = diff.length();
      Ped::Tangle angle = this->getWalkingDirection().angleTo(diff);
      if (distance <= getVisionDistance() && abs(angle.toDegree()) <= getVisionAngleDeg()/2.0
          || distance <= DISTANCE_PED_MIN)
      {
            perceivedAttractions.append(attraction);
      }
   }
   return perceivedAttractions;
}


/*
 * Return true if current agent must give attention to obstacle
 */
bool Agent::giveAttentionTo(const Obstacle* obstacle) const
{
    if (this->perceivedObstacles.contains(obstacle))
    {
       Ped::Tvector closestPoint = obstacle->closestPoint(p);
       Ped::Tvector diff = closestPoint - p;
       Ped::Tangle angleDirectionObstacle = this->getWalkingDirection().angleTo(diff);
       double distance = diff.length()
             - this->getRadius(angleDirectionObstacle,0.0)
             - Obstacle::getObstacleThickness()/2;

       if (abs(angleDirectionObstacle.toDegree()) <= getAttentionAngleDeg()/2.0 && distance <= getAttentionDistance()
           || distance <= DISTANCE_PED_MIN)
       {
         return true;
       }
    }
    return false;
}
