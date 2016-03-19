#include "Aria.h"

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArAnalogGyro gyro(&robot);
  ArSonarDevice sonar;
  ArRobotConnector robotConnector(&parser, &robot);

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "gotoActionExample: Could not connect to the robot.");
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

  ArLog::log(ArLog::Normal, "gotoActionExample: Connected to robot.");

  robot.addRangeDevice(&sonar);
  robot.runAsync(true);

  // Make a key handler, so that escape will shut down the program
  // cleanly
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  //设置最大的旋转速度
  robot.setAbsoluteMaxRotVel(30);

  // Collision avoidance actions at higher priority
  ArActionStallRecover recover;
  ArActionBumpers bumpers;

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

  //Going to  goals for %d seconds, then cancelling goal and exiting.
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
