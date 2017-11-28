#include "Aria.h"
#include <bits/stdc++.h>

using namespace std;
vector<int> fx;
vector<int> fy;
double stx,sty,enx,eny;
int curi=0;

class Logger {
private:
  ArRobot *robot;
  ArTime lastLogTime;
  ArFunctorC<Logger> robotTaskFunc;
  void logTask();
public:
  Logger(ArRobot *r);
  ~Logger();
}; 

Logger::Logger(ArRobot *r) :
  robot(r),                              // store a pointer to the ArRobot object
  robotTaskFunc(this, &Logger::logTask)  // initialize the functor to be given added as an ArRobot 
                                         // user task with this instance and which method to  call
{
  // add our task functor to the robot object as a user task, 
  // to be invoked in every robot task cycle (approx. every 100ms):
  robot->addSensorInterpTask("Logger", 50, &robotTaskFunc);    
}

Logger::~Logger()
{
  // it is important to remove our task if this object is destroyed, otherwise 
  // ArRobot will hold an invalid ArFunctor pointer in its tasks list, resulting
  // in a crash when it tries to invoke it.
  robot->remSensorInterpTask(&robotTaskFunc);
}

// This is the method invoked as the user task
void Logger::logTask()
{
  if(lastLogTime.mSecSince() >= 1000)  // 1 second has passed since start or last log 
  {
     printf("%f %f\n", robot->getX(), robot->getY());
     
     lastLogTime.setToNow(); // reset timer
  }
}


int main(int argc, char **argv)
{
     ifstream myReadFile;
     fx.clear();
     fy.clear();
     myReadFile.open("path2.txt");
     if (myReadFile.is_open()) {
     	while (!myReadFile.eof()) {
		int tx,ty;
        	myReadFile >> tx >> ty;
		fx.push_back(tx*50);
		fy.push_back(ty*50);
        }
    }
    myReadFile.close();
    for(int i=0;i<fx.size();i++)
	cout<<fx[i]<<" "<<fy[i]<<"\n";
    stx=fx[0];sty=fy[0];
    enx=fx[fx.size()-1];eny=fy[fx.size()-1];
    cout<<stx<<" "<<sty<<" "<<enx<<" "<<eny;

    Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArAnalogGyro gyro(&robot);
  ArSonarDevice sonar;
  ArRobotConnector robotConnector(&parser, &robot);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);


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

  //robot.addRangeDevice(&sonar);
  robot.runAsync(true);

  // Make a key handler, so that escape will shut down the program
  // cleanly
 /* ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");
*/
  // Collision avoidance actions at higher priority
    ArActionLimiterForwards limiterAction("speed limiter near", 300, 600, 250);
    ArActionLimiterForwards limiterFarAction("speed limiter far", 300, 1100, 400);
    ArActionLimiterTableSensor tableLimiterAction;
    robot.addAction(&tableLimiterAction, 100);
    robot.addAction(&limiterAction, 95);
    robot.addAction(&limiterFarAction, 90);

  // Goto action at lower priority
  ArActionGoto gotoPoseAction("goto", ArPose(stx, sty, 0), 600, 400, 40, 2);
  robot.addAction(&gotoPoseAction, 50);  

  // Stop action at lower priority, so the robot stops if it has no goal
  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 40);

  ArPose pos,pos1;
  pos.setPose(stx,sty,0);
    
  robot.lock();
  robot.moveTo (pos);       
  robot.unlock();
  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();
  int flag1=1;
  int count[fx.size()];
  for(int i=0;i<fx.size();i++)
	count[i]=0;  

  while (Aria::getRunning()) 
  {
    robot.lock();
    // achieved the previous goal.
    double cx,cy;
    cx=robot.getX();
    cy=robot.getY();
    double mind=INT_MAX;
    int minid=curi;
    for(int i=curi;i<min(curi+5,(int)fx.size());i++){
	double dist = sqrt(((fx[i]-cx)*(fx[i]-cx)) + ((fy[i]-cy)*(fy[i]-cy))); 
	cout<<count[i]<<" (";
	if(dist<mind&&count[curi]<75){
		mind=dist;
		minid = i;
	}
    }	
    count[curi]++;
    curi=minid;
    double ang = atan2((fy[minid]-cy),(fx[minid]-cx))*(180/3.14);
    double rotv = ang*(mind/10000);
    double lv = 1600/mind;
    if(lv>500)
	lv=500;
    if(lv<150)
	lv=150;
    cout<<"\n*"<<cx<<" "<<cy<<" "<<rotv<<" "<<minid<<" "<<ang<<" "<<mind;
    robot.setRotVel(rotv);
    robot.setVel(lv);	 
/*   if(flag1==1){
    	robot.setDeltaHeading(ang);
    	robot.setVel(0);
	flag1=2;
    }
    else{
	robot.setDeltaHeading(0);
    	robot.setVel(lv);
	flag1=1;
    }
*/    
    if(sqrt(((cx-enx)*(cx-enx))+((cy-eny)*(cx-eny)))<500)
	break;
    robot.unlock();
    ArUtil::sleep(50);
    
  }
  
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;

}

