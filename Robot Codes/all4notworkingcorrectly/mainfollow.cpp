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


int main(int argc, char **argv)
{
     ifstream myReadFile,myReadFile2;
     fx.clear();
     fy.clear();
     myReadFile.open("robotpos.txt");
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
  cout<<"$$$";
  
  double tmpx,tmpy;
  while (Aria::getRunning()) 
  {
    cout<<"---";
    robot.lock();
    // achieved the previous goal.
    double cx,cy;
    cx=robot.getX();
    cy=robot.getY();
    ofstream newFile("position1.txt");

    if(newFile.is_open())   
    {
        newFile << cx << " " << cy<<"\n";            
    }
    else 
    {
        //You're in trouble now Mr!
    }


    newFile.close();
    
 
      // Find location of ghost robot	
     ifstream myReadFile1;
     myReadFile1.open("position.txt");
     if (myReadFile1.is_open()) {
	int tx,ty;
	cout<<"READING";
        myReadFile1 >> tx >> ty;
	cout<<tx<<" "<<ty;
	tmpx = (double)tx;
	tmpy = (double)ty;
    }
    myReadFile1.close();
    cout<<"****";
    cout<<"\nMain Robot :"<<tmpx<<" "<<tmpy<<"\n";
    cout<<"\nFollower Robot :"<<cx<<" "<<cy<<"\n";
    double Frx=0,Fry=0,Ffinal,thetaf,att=10,rep=5000,Frx1=0,Fry1=0,Ffinal1;
    double distg = sqrt(((cx-tmpx)*(cx-tmpx))+((cy-tmpy)*(cy-tmpy)))/100;
    double thetag = ((atan2(tmpy-cy,tmpx-cx)*180/PI)-robot.getTh());

    cout<<"Distance :"<<distg<<"\n";
    getSonar(&robot);
    //Calculations for potential field
    int flag = 0;
    for(int i=0;i<8;i++){
	if(obsr[i]!=0){
                cout<<"\n^"<<i<<" "<<(rep*cos(obsang[i] * PI / 180.0))<<" "<<obsr[i]<<"^\n";
		if((obsr[i]*100)<400){
	                Frx-=((rep*cos(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
		    	Fry-=((rep*sin(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
			flag=1;
		}
	}
    }
    
    //Calculations for ghost robot
    if((distg*100)>350){    
    	Frx+=(att*cos(thetag*PI/180)*distg);
    	Fry+=(att*sin(thetag*PI/180)*distg);
    }
    else{
	Frx-=(100*rep*cos(thetag*PI/180)/distg);
    	Fry-=(100*rep*sin(thetag*PI/180)/distg);
	flag=1;
    }
    Ffinal1 = sqrt((Frx*Frx)+(Fry*Fry));
    thetaf = atan2(Fry,Frx)*180/PI;
    double lvel,rotvel;
    lvel = abs(Ffinal1);
    rotvel = thetaf*.5 ;

    distg = distg*100;    
    cout<<"Distance :"<<distg<<"\n";
    if(lvel>250)
	lvel = 250;
    if(flag==1)	
	lvel = 100;
    if(distg>600&&distg<=750){
	lvel = 10;
	rotvel = 0;
    }
    else if(distg<600){
	lvel = 0;
	rotvel = 0;
    }
    ofstream newFile1("distance12.txt");

    if(newFile1.is_open())   
    {
        newFile1 << distg<<"\n";            
    }
    else 
    {
        //You're in trouble now Mr!
    }
    newFile1.close();
    cout<<"\n#"<<lvel<<" "<<rotvel<<"\n";
    if(rotvel>30)
	rotvel = 30;
    if(rotvel<-30)
	rotvel = -30;
    myReadFile2.open("distance23.txt");
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
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;

}

