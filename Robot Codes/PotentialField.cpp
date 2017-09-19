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
     ifstream myReadFile;
     fx.clear();
     fy.clear();
     myReadFile.open("pathpf.txt");
     if (myReadFile.is_open()) {
     	while (!myReadFile.eof()) {
		int tx,ty;
        	myReadFile >> tx >> ty;
		fx.push_back(tx*50);
		fy.push_back(ty*50);
        }
    }
    myReadFile.close();
  //  for(int i=0;i<fx.size();i++)
//	cout<<fx[i]<<" "<<fy[i]<<"\n";
    stx=fx[0];sty=fy[0];
    enx=fx[fx.size()-1];eny=fy[fx.size()-1];
  //  cout<<stx<<" "<<sty<<" "<<enx<<" "<<eny;

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

  robot.enableMotors();

  ArSensorReading* Range;
  double reading, readingAngle;
  while (Aria::getRunning()) 
  {
    
   // robot.setVel(400);
    double cx=robot.getX(),cy=robot.getY();
    double distg = sqrt(((cx-enx)*(cx-enx))+((cy-eny)*(cy-eny)))/100;
    double thetag = ((atan2(eny-cy,enx-cx)*180/PI)-robot.getTh());
    cout<<"\nRobot Angles :"<<(atan2(eny-cy,enx-cx)*180/PI)<<" "<<robot.getTh()<<"\n";
    getSonar(&robot);
    robot.lock();
    double Frx=0,Fry=0,Ffinal,thetaf;
    double rep=5000,att=10;
    int flag = 0;
    for(int i=0;i<8;i++){
	if(obsr[i]!=0){
                cout<<"\n^"<<i<<" "<<(rep*cos(obsang[i] * PI / 180.0))<<" "<<obsr[i]<<"^\n";
		if((obsr[i]*100)<800){
			flag=1;
		}
                Frx-=((rep*cos(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
	    	Fry-=((rep*sin(obsang[i] * PI / 180.0))/(obsr[i]*obsr[i]));
	}
    }
    cout<<Frx<<" "<<Fry<<"#\n";
    cout<<"\nThetag = "<<thetag<<"\n";
    Frx+=(att*cos(thetag*PI/180)*distg);
    Fry+=(att*sin(thetag*PI/180)*distg);
    Ffinal = sqrt((Frx*Frx)+(Fry*Fry));
    thetaf = atan2(Fry,Frx)*180/PI;
    double lvel,rotvel;
    cout<<Frx<<" "<<Fry<<" "<<distg<<"$\n";
    lvel = abs(Ffinal);
    rotvel = thetaf*.5 ;
    cout<<"\n#"<<lvel<< " " <<rotvel <<" %\n";
    if(lvel>800)
	lvel = 800;
    if(rotvel>40)
	rotvel = 40;
    if(rotvel<-40)
	rotvel = -40;
   // lvel = 0;
   // rotvel = 20;
    if(flag==1)
	lvel=40;
    robot.setVel(lvel);
    robot.setRotVel(rotvel); 
    robot.unlock();
    ArUtil::sleep(200);
    robot.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    robot.unlock();
    if(sqrt(((cx-enx)*(cx-enx))+((cy-eny)*(cx-eny)))<500)
	break;
  }
  
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;

}
