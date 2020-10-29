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

#ifndef _agentgroup_h_
#define _agentgroup_h_

#include <pedsim/ped_vector.h>
#include <pedsim_simulator/element/scenarioelement.h>
#include <QGraphicsEllipseItem>
#include <QGraphicsItemGroup>
#include <QGraphicsLineItem>
#include <QTimer>

// Forward Declarations
class Agent;

class AgentGroup : public ScenarioElement
{
  Q_OBJECT

  // Constructor and Destructor
public:
   enum AgentGroupType
   {
     COUPLE =     0,
     FRIENDS =    1,
     FAMILY =     2,
     COWORKERS =  3
   };

  AgentGroup();
  explicit AgentGroup(const QList<Agent*>& agentsIn);
  explicit AgentGroup(std::initializer_list<Agent*>& agentsIn);
  virtual ~AgentGroup();

  // Signals
signals:
  void membersChanged();
  void memberAdded(int id);
  void memberRemoved(int id);

  // Slots
public slots:
  void onPositionChanged(double x, double y);

  // Static Methods
public:
  static QList<AgentGroup*> divideAgents(const QList<Agent*>& agentsIn);
  // → Helper
protected:
  static void reportSizeDistribution(const QVector<int>& sizeDistributionIn);

  // Methods
  // → Group members
public:
  QList<Agent*>& getMembers();
  const QList<Agent*>& getMembers() const;
  bool addMember(Agent* agentIn);
  bool removeMember(Agent* agentIn);
  bool setMembers(const QList<Agent*>& agentsIn);
  bool isEmpty() const;
  int memberCount() const;

  // → Mean walking direction
public:
  Ped::Tvector getWalkingDirection() const;

  // → Center of Mass
public:
  // 	QPointF getCenterOfMass() const;
  Ped::Tvector getCenterOfMass() const;
protected slots:
  Ped::Tvector updateCenterOfMass();

  // → Recollection
public:
  void setRecollect(bool recollectIn);
  bool isRecollecting() const;
  double getMaxDistance();
  static AgentGroupType generateGroupType(double random, int groupSize, QVector<int> sizeDistribution);
  const Agent* getLeftMost() const;
  const Agent* getClosestRight(const Agent* agent)const;
  const void computeDistanceAngleBetween(const Agent* agent, const Agent* other, double &distance, double &angle);
  Agent* getLeader();

  int getId()
  {
    return id_;
  }

  void setType(AgentGroupType typeIn)
  {
    type = typeIn;
  };

  AgentGroupType getType() const
  {
    return type;
  };

  void setLeader(Agent* leader)
  {
    this->leader = leader;
  };

protected:
  void updateMaxDistance();

  // → ScenarioElement Overrides
public:
  virtual QString toString() const;

  // Attributes
protected:
  QList<Agent*> members;
  Agent* leader;
  AgentGroupType type;

  // → Center of Mass
  bool dirty;
  Ped::Tvector cacheCoM;
  //   → delayed update
  QTimer comUpdateTimer;

  // → recollecting group
  bool recollecting;
  bool dirtyMaxDistance;
  double cacheMaxDistance;

  int id_;
};

#endif
