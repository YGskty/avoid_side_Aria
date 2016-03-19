
#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArActionAvoidSide.h"
#include "ArRobot.h"

/**
   @param name name of the action
   @param obstacleDistance distance at which to start avoiding (mm)
   @param turnAmount degrees at which to turn (deg)
*/

AREXPORT ArActionAvoidSide::ArActionAvoidSide(const char *name,
                                        	ArPose goal,
					      					double obstacleFrontDistance,
                                        	double obstacleSideDistance,
					        				double turnAmount) :
  ArAction(name, "Avoids side obstacles, ie walls")
{
  setNextArgument(ArArg("goal", &myGoal, "ArPose to go to. (ArPose)"));
  myGoal = goal;
  myOldGoal = myGoal;
  setNextArgument(ArArg("obstacle front distance", &myObsDist_front, 
			"Distance at which to start avoiding (mm)"));
  myObsDist_front = obstacleFrontDistance;
  setNextArgument(ArArg("obstacle side distance", &myObsDist_side, 
			"Distance at which to start avoiding (mm)"));
  myObsDist_side = obstacleSideDistance;
  setNextArgument(ArArg("turn amount", &myTurnAmount,
			"Degrees at which to turn (deg)"));
  myTurnAmount = turnAmount;                     

  myTurning = 0; //1:turn left  -1 turn right   0 :go ahead

  //雷达数据大突变，保持
  leftDist_before = 0;
  rightDist_before = 0;

  //标志区
  //到达目标，停止机器人标志，仅使用一次
  STOP_CAR_FIRST = false;

  //新定义目标点后，车体指向目标点完毕标志
  ACHIEVE_ANGLE = false;

  //转向目标点标志，仅使用一次
  TURN_TO_GOAL = false;

  //当前位置到目标位置角度
  poseAngle = 0;
}

AREXPORT ArActionAvoidSide::~ArActionAvoidSide()
{

}

AREXPORT bool ArActionAvoidSide::haveAchievedGoal(void)
{
  if (myState == STATE_ACHIEVED_GOAL)
    return true;
  else
    return false;
}

AREXPORT void ArActionAvoidSide::cancelGoal(void)
{
  myState = STATE_NO_GOAL;
}

AREXPORT void ArActionAvoidSide::setGoal(ArPose goal)
{
  myState = STATE_GOING_TO_GOAL;
  myOldGoal = myGoal;
  myGoal = goal;
  //myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
}



AREXPORT ArActionDesired *ArActionAvoidSide::fire(
	ArActionDesired currentDesired)
{
	if(myGoal.findDistanceTo(myOldGoal) > 5){
		setGoal(myGoal);
	} 
	poseAngle = myRobot->getPose().findAngleTo(myGoal);
	//重新设定目标点后进行方位对准
	if(!ACHIEVE_ANGLE){
		//poseAngle = myRobot->getPose().findAngleTo(myGoal);
		if(ArMath::fabs(poseAngle - myRobot->getTh())>1){
			myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
			printf("try to pose to the target1, angle is %f \n", myRobot->getPose().findAngleTo(myGoal));
		}
		else{
			ACHIEVE_ANGLE = true;
			printf("ACHIEVE_ANGLE\n");
		}
		return &myDesired;
	}
	

    // if we're there we don't do anything
  	if (myState == STATE_ACHIEVED_GOAL || myState == STATE_NO_GOAL){
  		return NULL;
  	}
    
  double leftDist, rightDist;
  double dist, angle;

  //小车距离目标点的距离
  double dist_to_goal;

  double left_err;
  double right_err;

  double angle_attractive;

  double Vl, Vr;

  double k1 = 0.27;  //0.2

  //较远距离找目标点调整参数，有关角度
  double k2 = 1.3; // 1.8 
  //较远距离找目标点调整参数，有关距离
  double kp2 = 0.10; // 0.1

  //较近距离找目标点调整参数，有关角度
  //double kp3_angle = 0.8;
  //较近距离找目标点调整参数，有关距离
  double kp3_dist = 1.0;  // 0.6 //2

  //进入目标点的误差
  double err_goal = 0;

  double  car_val_normal = 250; //300
  double  car_val_turn = 200; //150;

  dist = myRobot->checkRangeDevicesCurrentPolar(-40, 40, &angle);

  dist_to_goal = myRobot->getPose().findDistanceTo(myGoal);
	  //- myRobot->getRobotRadius());
  leftDist = myRobot->checkRangeDevicesCurrentPolar(60, 120);
      //- myRobot->getRobotRadius());
  rightDist = myRobot->checkRangeDevicesCurrentPolar(-120, -60);
	      //myRobot->getRobotRadius());
  
  if(leftDist >4000){
	leftDist = myRobot->checkRangeDevicesCurrentPolar(20, 120) ;
      //myRobot->getRobotRadius());
  }

  if(rightDist >4000){
	rightDist = myRobot->checkRangeDevicesCurrentPolar(-120, -20) ;
      //myRobot->getRobotRadius());
  }


  if(dist >1000){
    dist = myRobot->checkRangeDevicesCurrentPolar(-70, 70, &angle) ;
	  //- myRobot->getRobotRadius());
  }

  angle_attractive = ArMath::fabs(ArMath::subAngle(myRobot->getPose().findAngleTo(myGoal), myRobot->getTh()));

/* 
  printf("findAngleTomyGoal = %f \n myRobot->getTh() = %f \n angle_attractive = %f \n sonar angle = %f \n", \
  	      myRobot->getPose().findAngleTo(myGoal), myRobot->getTh(), angle_attractive, angle);
*/

  if(angle_attractive>90){
  	angle_attractive = 180 -angle_attractive;
  }
  
/*
  printf("##\n");
  printf("original sonar dist =  %f\n",dist);
  printf("distacne between my pose to goal =  %f\n",dist_to_goal);
  printf("original leftDist =  %f\n",leftDist);
  printf("original rightDist =  %f\n",rightDist);
  printf("original angle =  %f\n\n",angle);

  printf("important information .....：\n");
  printf("angle_attractive  =  %f\n",angle_attractive);
  printf("my position Y =  %f\n",myRobot->getPose().getY());
  printf("goal position y =  %f\n",myGoal.getY());
  printf("###################################\n");
*/
  //屏蔽周围过大环境,初始设置为1800mm
  if(leftDist>1800){
  	leftDist = 1800;
  }
  if(rightDist>1800){
  	rightDist = 1800;
  }
  //减去设定的边缘安全距离
  left_err = leftDist - myObsDist_side;
  right_err = rightDist - myObsDist_side;
  myDesired.reset();

  printf("\n\n**");
  printf("dist = %f \n", dist);
  printf("distacne between my pose to goal =  %f\n",dist_to_goal);
  printf("left_err = %f \n", left_err);
  printf("right_err = %f \n", right_err);
  printf("angle_attractive = %f \n", angle_attractive);
  printf("poseAngle = %f \n", poseAngle);
  printf("*******************************\n\n");

//到达目标点做相应处理
if(dist_to_goal<600){
	if(dist_to_goal<200){
		//myRobot->clearDirectMotion();
		if(!STOP_CAR_FIRST){
			myRobot->stop();
			STOP_CAR_FIRST = true;
			printf("STOP_CAR_FIRST = true\n");
		}
		//myRobot->setHeading(0);
		//myRobot->setHeading(180);
		//myRobot->setRotVel(45);
		//myRobot->setDirectMotionPrecedenceTime(50);
		//myRobot->clearDirectMotion();
		printf("achieved the goal\n");
		myState = STATE_ACHIEVED_GOAL;
		STOP_CAR_FIRST = false;
		ACHIEVE_ANGLE = false;
		return &myDesired;
	}
	if(ArMath::fabs(poseAngle - myRobot->getTh())>5){
		if(!TURN_TO_GOAL){
			myRobot->stop();
			TURN_TO_GOAL = true;
			printf("TURN_TO_GOAL = true\n");
		}
		myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
		printf("try to pose to the target2, angle is %f \n", myRobot->getPose().findAngleTo(myGoal));
		printf("当前位姿信息= %f\n：",myRobot->getTh());
	}
	else{
		TURN_TO_GOAL = false;
		err_goal = dist_to_goal-200;
		Vl = kp3_dist*err_goal;
		Vr = kp3_dist*err_goal;
		if(Vl<100){
			Vl = Vr = 100;
		}
		myRobot->setVel2(Vl, Vr);
		printf("near the goal\n");
		printf("V = %F \n", Vr);
		printf("already pose to the target \n");
	}
	return &myDesired;
}


if(dist>myObsDist_front){
	
	if((right_err<0)&&(left_err<0)){
		myRobot->setVel2(-100, -100); 
		printf("front distacne is large, but the left and right distacne is too narrow, go back \n");
	}
	else if((right_err < 0)&&(left_err>0))
	{
		Vl = car_val_turn - k1*rightDist;
		Vr = car_val_turn + k1*rightDist;
		myRobot->setVel2(Vl, Vr);
		printf("front distacne is large, go left\n");
	}
	else if((left_err < 0)&&(right_err>0))
	{
		Vl = car_val_turn + k1*leftDist;
		Vr = car_val_turn - k1*leftDist;
		myRobot->setVel2(Vl, Vr);
		printf("front distacne is large, go right\n");
	}
	else{
		//避障之后机器人在目标位置的左边,偏移一个容错距离300，右转追寻目标点
		if((((myRobot->getPose().getY() - myGoal.getY())>270)&&(ArMath::fabs(poseAngle)<=90)) \
			||(((myRobot->getPose().getY() - myGoal.getY())<-270)&&(ArMath::fabs(poseAngle)>=90))){ //100
			Vl = car_val_turn + kp2*((myRobot->getPose().getY() - myGoal.getY())) + k2*angle_attractive;
			Vr = car_val_turn - kp2*((myRobot->getPose().getY() - myGoal.getY())) - k2*angle_attractive;
			myRobot->setVel2(Vl, Vr);
			printf("go for the goal, turn right\n");
		}
		//避障之后机器人在目标位置的右边，偏移一个容错距离100，左转追寻目标点
		else if((((myRobot->getPose().getY() - myGoal.getY())>270)&&(ArMath::fabs(poseAngle)>90)) \
			||(((myRobot->getPose().getY() - myGoal.getY())<-270)&&(ArMath::fabs(poseAngle)<90))){ //100
			Vl = car_val_turn + kp2*((myRobot->getPose().getY() - myGoal.getY())) - k2*angle_attractive;
			Vr = car_val_turn - kp2*((myRobot->getPose().getY() - myGoal.getY())) + k2*angle_attractive;
			myRobot->setVel2(Vl, Vr);
			printf("go for the goal, turn left\n");

		}
		//目标点在容错方位范围内，切换为快速直奔目标点
		else{
			//car_val_normal = 300;
			Vl = Vr = car_val_normal;
			myRobot->setVel2(Vl, Vr);
			printf("no obstacle,go head\n");
		}
	}
	return &myDesired;
}

if(dist>myObsDist_front*0.8){
	car_val_normal = 250*dist/myObsDist_front;
 	if (car_val_normal > 250){
    	car_val_normal = 250;
 	}
	if((right_err<0)&&(left_err<0)){
		myRobot->setVel2(-50, -50); 
		printf("near obstacle, but the left and right distacne is too narrow,go back \n");
	}
	else if ((right_err < 0)&&(left_err>0))
	{
		Vl = car_val_turn - k1*rightDist;
		Vr = car_val_turn + k1*rightDist;
		myRobot->setVel2(Vl, Vr);
		printf("near obstacle, go left\n");
	}
	else if ((left_err < 0)&&(right_err>0))
	{
		Vl = car_val_turn + k1*leftDist;
		Vr = car_val_turn - k1*leftDist;
		myRobot->setVel2(Vl, Vr);
		printf("near obstacle, go right\n");
	}
	else{
		Vl = Vr = car_val_normal;
		myRobot->setVel2(Vl, Vr);
		printf("near obstacle,slow down\n");
	}
	return &myDesired;
}

  if(dist >0.4*myObsDist_front){
  	/*
  	  if(ArMath::fabs(leftDist -leftDist_before)>1700){
  	  	leftDist = leftDist_before;
  	  }
  	  if(ArMath::fabs(rightDist -rightDist_before)>1700){
  	  	rightDist = rightDist_before;
  	  }
  	  */

      if(leftDist>rightDist){
      	rightDist = myRobot->checkRangeDevicesCurrentPolar(-190, -20);
      	if(rightDist>800){
      		rightDist = 800;
      	}
        Vl = car_val_turn - k1*rightDist ;//+ k2*angle_attractive;
        Vr = car_val_turn + k1*rightDist ;//- k2*angle_attractive;
        myRobot->setVel2(Vl, Vr);
        printf("turn left\n");
      }
      else{
      	leftDist = myRobot->checkRangeDevicesCurrentPolar(20, 190);
      	if(leftDist>800){
      		leftDist = 800;
      	}
        Vl = car_val_turn + k1*leftDist;  //- k2*angle_attractive;
        Vr = car_val_turn - k1*leftDist;  //+ k2*angle_attractive;
        myRobot->setVel2(Vl, Vr);
        printf("turn right\n");
      }
  }else if(dist <= 0.4*myObsDist_front){
    	if((left_err > right_err)&&(right_err > -100)){
    		myRobot->setVel2(-70, 70);
    		printf("too close to the obstacle ,try to escape and turn left\n");
    	}
    	else if ((right_err > left_err)&&(left_err > -100))
    	{
    		myRobot->setVel2(70,-70);
    		printf("too close to the obstacle ,try to escape and turn right\n");
    	}
    	else{
    		myRobot->setVel2(-70, -70);
  			printf("too close to the obstacle ,go back -_-!\n");
    	}
  }

  leftDist_before = leftDist;
  rightDist_before = rightDist;
  return &myDesired;
}
