/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#ifndef ARACTIONAVOIDSIDE_H
#define ARACTIONAVOIDSIDE_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/// Action to avoid impacts by firening into walls at a shallow angle
/**
   This action watches the sensors to see if it is close to firening into a wall
   at a shallow enough angle that other avoidance may not avoid.

  @ingroup ActionClasses
*/
class ArActionAvoidSide : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionAvoidSide(const char *name = "Avoid side", 
                  ArPose goal = ArPose(0.0, 0.0, 0.0), 
		              double obstacleFrontDistance = 450,
                  double obstacleSideDistance = 300,
		              double turnAmount = 5);
  /// Destructor
  AREXPORT virtual ~ArActionAvoidSide();

  /** Sees if the goal has been achieved. The goal is achieved when
   *  the robot's repordet position is within a certain distance
   *  (given in the constructor or in setCloseDist) from the goal pose. */
  AREXPORT bool haveAchievedGoal(void);

  /** Cancels the goal; this action will stop requesting movement. However,
   *  any currently requested motion (either previously requested by this
   *  action or by another action) will continue to be used. Use an ArActionStop
   *  action (activate it, or set it at a lower priority) to stop the robot.
   */
  AREXPORT void cancelGoal(void);

  /// Sets a new goal and sets the action to go there
  AREXPORT void setGoal(ArPose goal);

  AREXPORT virtual ArActionDesired * fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArPose myGoal;
  double myObsDist_front;
  double myObsDist_side;
  double myTurnAmount;
  int myTurning;
  ArActionDesired myDesired;
  ArPose myOldGoal;
  double leftDist_before;
  double rightDist_before;
  volatile bool STOP_CAR_FIRST;
  volatile bool ACHIEVE_ANGLE;
  volatile bool TURN_TO_GOAL;
  double poseAngle;
  
  enum State
  {
    STATE_NO_GOAL, 
    STATE_ACHIEVED_GOAL,
    STATE_GOING_TO_GOAL
  };
  State myState;

};

#endif // ARACTIONAVOIDSIDE_H
