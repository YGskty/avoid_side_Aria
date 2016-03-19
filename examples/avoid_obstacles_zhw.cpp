#include "Aria.h"
/**
   @功能：机器人避障及导航代码，基于超声波数据，目标是到达目标点，在过程中需要规避
        障碍物。到达目标点之后传入新的目标点，继续追寻下一个目标点，直到
        所有目标点都完成。该部分为主函数，负责上层调度，底层算法及实现看核心文件 /src/ArActionAvoidSide.cpp文件
   @author：zhw
   @email:zhw793984895@163.com
   @time:2016-3-17
   @version:v1.1
 */
int main(int argc, char **argv)
{ 
  //一些Aria 定义的初始化步骤，按葫芦画瓢
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArAnalogGyro gyro(&robot);
  ArSonarDevice sonar;
  ArRobotConnector robotConnector(&parser, &robot);

  //连接机器人
  if(!robotConnector.connectRobot())
  {
    //ArLog::log(ArLog::Terse, "gotoGoal: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  //ArLog::log(ArLog::Normal, "gotoGoal: Connected to robot.");
  
  robot.addRangeDevice(&sonar);
  robot.runAsync(true);

  // 定义一个快捷键，按键盘Esc小车停止
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  //设置最大的旋转速度
  robot.setAbsoluteMaxRotVel(30);

  // Collision avoidance actions at higher priority
  ArActionStallRecover recover;
  ArActionBumpers bumpers;
  //引入避障action,前方安全距离为1000mm,侧边安全距离为340mm
  ArActionAvoidSide avoidSide("Avoid side", ArPose(0, 0), 1000, 340, 1);

  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 95);
  robot.addAction(&avoidSide, 80);

  // Goto action at lower priority
  //ArActionGoto gotoPoseAction("goto");
  //robot.addAction(&gotoPoseAction, 60);

  // Stop action at lower priority, so the robot stops if it has no goal
  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 50);


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  //设置一个时间，到了该时间程序就退出，120s 退出 
  const int duration = 120000; //msec
  
  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();

  while (Aria::getRunning()) 
  {
    robot.lock();

    // Choose a new goal if this is the first loop iteration, or if we 
    // achieved the previous goal.
    if (first || avoidSide.haveAchievedGoal())
    {
      //avoidSide.cancelGoal();
      first = false;
      goalNum++;
      printf("count goalNum = %d\n", goalNum);
      if (goalNum > 4){
        goalNum = 1; // start again at goal #1
      }

      // set our positions for the different goals
      // 设置目标点
      if (goalNum == 1){
        avoidSide.setGoal(ArPose(3800, 0));
        printf("goalNum == 1\n\n");
      }
      else if (goalNum == 2){
        avoidSide.setGoal(ArPose(0, 0));
        printf("goalNum == 2\n\n");
      }
      else if (goalNum == 3){
        avoidSide.setGoal(ArPose(3800, 0));
        printf("goalNum == 3\n\n");
      }
      else if (goalNum == 4){
        avoidSide.setGoal(ArPose(0, 0));
        printf("goalNum == 4\n\n");
      }
    }
    if(start.mSecSince() >= duration) {
      ArLog::log(ArLog::Normal, "%d seconds have elapsed. Cancelling current goal, waiting 3 seconds, and exiting.", duration/1000);
      avoidSide.cancelGoal();
      robot.unlock();
      ArUtil::sleep(3000);
      break;
    }
    robot.unlock();
    ArUtil::sleep(100);
  }
    // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}
