#include "Aria.h"
#include <bits/stdc++.h>
#define PI 3.14159265
using namespace std;
vector<int> fx;
vector<int> fy;
double stx,sty,enx,eny;
int curi=0;
int obsr[8],obsang[8];

void getSonar(ArRobot *thisRobot)
{
	int numSonar;
	int i;
	int x,y,th;
	x = (int)thisRobot->getX();
	y = (int)thisRobot->getY();
	th = (int)thisRobot->getTh();
	numSonar = thisRobot->getNumSonar(); //Get number of sonar
	ArSensorReading* sonarReading;
	//To hold each reading
	//cout<<"Position of robot is "<<x<<" "<<y<<"\n";
	for (i = 0; i < numSonar; i++)
	//Loop through sonar
	{
		sonarReading = thisRobot->getSonarReading(i);
		int range = sonarReading->getRange();
		int ang = sonarReading->getSensorTh();
		//obsx[i] = x + range*cos(ang * PI / 180.0);
		//obsy[i] = y + range*sin(ang * PI / 180.0);
		if(range>=1200)
			obsr[i]=0;
		else
			obsr[i]=range/100;	
		obsang[i] = ang;	
//		//Get each sonar reading
		//cout << "Sonar reading " << i << " = " << sonarReading->getRange()<< " Angle " << i << " = " <<sonarReading->getSensorTh() << "\n";
		//cout << "Position of Obstacle " << i << " " << obsx[i] << "\n";		
		//cout << "Position of Obstacle " << i << " " << obsx[i] << " " << obsy[i] <<"\n"; 
	}
}


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
     ifstream myReadFile,myReadFile2;
     fx.clear();
     fy.clear();
     myReadFile.open("final4.txt");
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

  robot.runAsync(true);

  // Make a key handler, so that escape will shut down the program
  // cleanly
  ArKeyHandler keyHandler;
  ArActionKeydrive keydriveAct;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  robot.addRangeDevice(&sonar);
  robot.addAction(&keydriveAct,45);
  printf("You may press escape to exit\n");

  ArPose pos,pos1;
  pos.setPose(stx,sty,0);
    
  robot.lock();
  robot.moveTo (pos);       
  robot.unlock();
  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  bool first = true;
  ArTime start;
  start.setToNow();
  int flag1=1;
  curi=0;
  
  ArSensorReading* Range;
  double reading, readingAngle;

  int counter=0;
  while (Aria::getRunning()) 
  {
     double tmpx1,tmpy1;
     ifstream myReadFile1;
     myReadFile1.open("position.txt");
     if (myReadFile1.is_open()) {
	int tx,ty;
	cout<<"READING";
        myReadFile1 >> tx >> ty;
	cout<<tx<<" "<<ty;
	tmpx1 = (double)tx;
	tmpy1 = (double)ty;
    }
    myReadFile1.close();
    
    robot.lock();
    counter=(counter+1)%20;
    // achieved the previous goal.
    double cx,cy;
    cx=robot.getX();
    cy=robot.getY();
    ofstream newFile("position.txt");

    if(newFile.is_open())   
    {
        newFile << cx << "\n" << cy<<"\n";            
    }
    else 
    {
        //You're in trouble now Mr!
    }


    newFile.close();
    // Find location of ghost robot
    int index=-1;
    double tmpx,tmpy;
    for(int i=curi;i<fx.size()&&i<curi+2;i++){
	index = i;
	tmpx=fx[i];
	tmpy=fy[i];	
	if(sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))<600){
		;
	}
	else
		break;
    } 
    cout<<"*"<<index<<" "<<curi<<"*\n";
    if(index>(curi+2)){
	index = curi + 1;
    }
    curi=index;
    if(index==fx.size()-1)
	break;
    if(index == -1)
	index = curi-1;
    else
	index--;
    cout<<"#"<<index<<" "<<curi<<"*\n";
    tmpx=fx[index];
    tmpy=fy[index];	
	
    double Frx=0,Fry=0,Ffinal,thetaf,att=10,rep=5000,Frx1=0,Fry1=0,Ffinal1;
    double distg = sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))/100;
    double thetag = ((atan2(tmpy-cy,tmpx-cx)*180/PI)-robot.getTh());

    //Calculations for ghost robot    
    Frx1+=(att*cos(thetag*PI/180)*distg);
    Fry1+=(att*sin(thetag*PI/180)*distg);
    Ffinal1 = sqrt((Frx1*Frx1)+(Fry1*Fry1));
    thetaf = atan2(Fry1,Frx1)*180/PI;
    double lvel,rotvel;
    rotvel = thetaf*.5 ;
    getSonar(&robot);
    //Calculations for potential field
    int flag = 0;
    for(int i=0;i<8;i++){
	if(obsr[i]!=0){
                cout<<"\n^"<<i<<" "<<(rep*cos(obsang[i] * PI / 180.0))<<" "<<obsr[i]<<"^\n";
		if((obsr[i]*100)<400){
			flag=1;
		}
                Frx-=((rep*cos(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
	    	Fry-=((rep*sin(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
	}
    }
    Frx+=(att*cos(thetag*PI/180)*distg);
    Fry+=(att*sin(thetag*PI/180)*distg);
    double thetag1 = ((atan2(cy-tmpy1,cx-tmpx1)*180/PI)-robot.getTh());
    double distg1 = sqrt(((cx-tmpx1)*(cx-tmpx1))+((cy-tmpy1)*(cy-tmpy1)))/100;
    cout<<"Main "<<cx<<" "<<cy<< " Follow "<<tmpx1<<" "<<tmpy1<<" "<<distg1<<"\n";
    cout<<"\nForward Theta = "<<thetag<<" Pulling Back Theta = "<<thetag1<<"Robot Theta = "<< robot.getTh()<<" Distance "<<distg1<<"\n";
    distg1=distg;
    if(distg1>3.8){
	cout<<"Original Forces"<<Frx1<<" "<<Fry1<<"\n";
	cout<<distg1<<" Child Pulling Back\n";   
    	Frx+=(att*cos(thetag1*PI/180)*distg1);
    	Fry+=(att*sin(thetag1*PI/180)*distg1);
	Frx1+=(att*cos(thetag1*PI/180)*distg1);
    	Fry1+=(att*sin(thetag1*PI/180)*distg1);
	cout<<"New Forces"<<Frx1<<" "<<Fry1<<"\n";
    }
    else if (distg1<3.2){
	if(distg1!=0){
		Frx-=(.01*rep*cos(thetag1*PI/180)/distg1);
    		Fry-=(.01*rep*sin(thetag1*PI/180)/distg1);
	}
    }
    lvel = abs(Ffinal1);
    cout<<"\nFlag = "<<flag<<"\n";
    Ffinal = sqrt((Frx*Frx)+(Fry*Fry));
    thetaf = atan2(Fry,Frx)*180/PI;
    distg = distg*100;
    if(flag==0){
    	/*if(distg>400&&distg<600)
		lvel = 100;
    	if(distg>0&&distg<=200)
		lvel = 100;
    	if(distg>200&&distg<=400)
		lvel = 400;
	*/
    	if(rotvel>30)
		rotvel = 30;
    	if(rotvel<-30)
		rotvel = -30;
    }
    distg/=100;
    if(flag==1){
	lvel = abs(Ffinal);
	if(lvel>100)
		lvel=100;
    	rotvel = thetaf*.5 ;
	if(distg>500){
		lvel=10;
		rotvel=0;
		if(counter==0)
			curi-=1;
	}
	else{
		int mindis = INT_MAX;
		int mindisid = curi;
		for(int i=curi-2;i<fx.size();i++){
			tmpx=fx[i];
			tmpy=fy[i];	
			if(sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))<mindis){
				mindis = sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)));
				mindisid = i;
			}
				else
					break;
    		} 
		if(mindis<500)
			curi = mindisid;
	}
    }
    
    myReadFile2.open("distance12.txt");
     if (myReadFile2.is_open()) {
	double t1;
        myReadFile2 >> t1;
        cout<<"*Reading Distance "<<t1<<"\n";
        if(t1>1100){
		lvel = 0;
		rotvel = 0;
    	}
    }
    myReadFile2.close();
    
    robot.setVel(lvel);
    robot.setRotVel(rotvel);
    robot.unlock();
    ArUtil::sleep(150);
    
  }
  robot.unlock();
  
  while (Aria::getRunning()) 
  {
    robot.lock();
    // achieved the previous goal.
    double cx,cy;
    cx=robot.getX();
    cy=robot.getY();
    ofstream newFile("position.txt");

    if(newFile.is_open())   
    {
        newFile << cx << " " << cy;            
    }
    else 
    {
        //You're in trouble now Mr!
    }


    newFile.close();
    // Find location of ghost robot
    int index=-1;
    double tmpx,tmpy;
    tmpx=fx[fx.size()-1];
    tmpy=fy[fx.size()-1];
    cout<<"#"<<cx<<" "<<cy<<" "<<tmpx<<" "<<tmpy<<"\n";	
    if(sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))<200){
	break;
    }
    double Frx=0,Fry=0,Ffinal,thetaf,att=10;
    double distg = sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))/100;
    double thetag = ((atan2(tmpy-cy,tmpx-cx)*180/PI)-robot.getTh());
    
    Frx+=(att*cos(thetag*PI/180)*distg);
    Fry+=(att*sin(thetag*PI/180)*distg);
    Ffinal = sqrt((Frx*Frx)+(Fry*Fry));
    thetaf = atan2(Fry,Frx)*180/PI;
    double lvel,rotvel;
    lvel = abs(Ffinal);
    rotvel = thetaf*.5 ;
    if(distg>600&&distg<800)
	lvel = 60;
    if(distg>0&&distg<=200)
	lvel = 60;
    if(distg>200&&distg<=600)
	lvel = 400;
    if(rotvel>30)
	rotvel = 30;
    if(rotvel<-30)
	rotvel = -30;
    robot.setVel(lvel);
    robot.setRotVel(rotvel);
    robot.unlock();
    ArUtil::sleep(150);
    
  }  
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;

}

