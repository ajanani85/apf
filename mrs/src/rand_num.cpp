#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std;

class RandClass
{
public:
	RandClass();
private:
	ros::NodeHandle nh;
	ros::Subscriber laser_sub;

	void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan);
};

RandClass::RandClass()
{
	laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/robot_0/base_scan",10,&RandClass::laserCB,this);
}

void RandClass::laserCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	srand (time(NULL));
	float randx = -1*rand()%60+30; //a random number between 5 and 15
	usleep(500000);
	srand (time(NULL));
	float randy = -1*(rand()%30+15); //a random number between -1 and 1
	float x = 0.0;
	float y = 0.0;
	ROS_INFO(" random Home location = %f %f",randx, randy);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rand_num");
	RandClass rc;
	ros::spin();
	return 0;
}
