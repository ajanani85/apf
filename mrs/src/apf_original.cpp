#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std;

#define PI 3.14159265359
#define Rad2Deg 57.2957795
#define Deg2Rad 0.0174532925
#define PITCH 0.3522504892367
#define DistanceTh 0.2
#define DistanceCrt 1.5
#define DistanceCrtTwo 0.5
#define MaxVel 0.5

void stop(void);
struct point
{
	float x;
	float y;
	float theta;
};
struct force
{
	float mag;
	float theta;
	float x;
	float y;
};
point target, current;

bool targetIsReached = false;

ros::Publisher vel_pub;
geometry_msgs::Twist vel;

class APFClass
{
public:
	APFClass();
private:
	ros::NodeHandle nh;
	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan);
	void poseCB(const nav_msgs::Odometry::ConstPtr &pose);
};

APFClass::APFClass()
{
	laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/robot_0/base_scan",10,&APFClass::laserCB,this);
	pose_sub = nh.subscribe<nav_msgs::Odometry>("/robot_0/base_pose_ground_truth",10,&APFClass::poseCB,this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",10);
	target.x = 0.0;
	target.y = 0.0;
}

void APFClass::laserCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	float beams[512];
	point dir, obj_new[512], obj[512];
	force attraction, repulsion[512], sum;
	int RotCoef = 1;
	sum.x = 0.0;
	sum.y = 0.0;
	int beamCounter = 1;
	dir.x = target.x - current.x;
	dir.y = target.y - current.y;
	dir.theta = atan2(dir.y,dir.x)*Rad2Deg;
	float distanceToTarget = sqrt(dir.x*dir.x + dir.y*dir.y);
	float R[2][2] = {{cos((90-current.theta)*Deg2Rad), -1*sin((90-current.theta)*Deg2Rad)},
									 {sin((90-current.theta)*Deg2Rad), cos((90-current.theta)*Deg2Rad)}};
	
	if (distanceToTarget >= DistanceTh)
	{		
		for (int i=0; i<=511; i++)
		{
			if (scan->ranges[i] < 4.5 )
			{
				beams[i] = scan->ranges[i];
				
				obj[i].x = beams[i]*cos((i*PITCH)*Deg2Rad);
				obj[i].y = beams[i]*sin((i*PITCH)*Deg2Rad);
				beamCounter++;
			}
			else if (scan->ranges[i] == 4.5)
			{
				beams[i] = 0.0;
				obj[i].x = 0.0;
				obj[i].y = 0.0;
			}
		}
		
		//form the orientation matrix
		for (int i=0; i<= 511; i++)
		{
			if (beams[i] > 0.0 && beams[i] < 4.5)
			{
				repulsion[i].x = obj[i].x;
				repulsion[i].y = obj[i].y;
				
				//remember: repulsion force is 180 degrees mirrored of the vector describing the object and the robot
				if (beams[i] <= DistanceCrt && beams[i] > DistanceCrtTwo)
				{
					repulsion[i].x = 20*obj[i].x;
					repulsion[i].y = 20*obj[i].y;
					RotCoef = 10;
				}
				if (beams[i] <= DistanceCrtTwo)
				{
					repulsion[i].x = 100*obj[i].x;
					repulsion[i].y = 100*obj[i].y;
					RotCoef = 100;
				}
				if (beams[i] > DistanceCrt)
				{
					repulsion[i].x = obj[i].x;
					repulsion[i].y = obj[i].y;
					RotCoef = 1;
				}
				repulsion[i].mag = 1/sqrt(repulsion[i].x*repulsion[i].x+repulsion[i].y*repulsion[i].y);
				repulsion[i].theta = atan2(repulsion[i].y,repulsion[i].x);
			}
			else if (beams[i] == 0.0)
			{
				repulsion[i].x = 0.0;
				repulsion[i].y = 0.0;
				repulsion[i].mag = 0.0;
				repulsion[i].theta = 0.0;
			}
			
			sum.x = sum.x + repulsion[i].x;
			sum.y = sum.y + repulsion[i].y;
		}
		//ROS_INFO("repulsion components 1: x:%f, y:%f, mag: %f, theta: %f",sum.x,sum.y, sum.mag, sum.theta);	
		
		
		float rx = sum.x*cos(180*Deg2Rad) - sum.y*sin(180*Deg2Rad);
		float ry = sum.x*sin(180*Deg2Rad) + sum.y*cos(180*Deg2Rad);		
		sum.x	= rx*R[0][0] - ry*R[0][1];
		sum.y = -rx*R[1][0] + ry*R[1][1];
		sum.mag = sqrt(sum.x*sum.x+sum.y*sum.y);
		sum.theta = Rad2Deg*atan2(sum.y,sum.x);
		
		//applying robot's orientation
		
		//ROS_INFO("repulsion components 2: x:%f, y:%f, mag: %f, theta: %f",sum.x,sum.y, sum.mag, sum.theta);	
		//attraction force
		attraction.mag = sqrt(dir.x*dir.x+dir.y*dir.y);
		attraction.theta = Rad2Deg*atan2(dir.y,dir.x);
		attraction.x = dir.x;
		attraction.y = dir.y;
		//ROS_INFO("attraction components: x:%f, y:%f, mag: %f, theta: %f",attraction.x,attraction.y, attraction.mag, attraction.theta);
		//sum force	
		int Krep = 7;
		int Katt = 20;
		sum.x = sum.x + Katt*attraction.x;
		sum.y = sum.y + Katt*attraction.y;
		sum.theta = Rad2Deg*atan2(sum.y,sum.x);
		sum.mag = sqrt(sum.x*sum.x+sum.y*sum.y);
		//ROS_INFO("sum force components: x:%f,y:%f -> mag:%f, angle:%f",sum.x,sum.y,sum.mag,sum.theta);
		
		/*Here is where you assign speed to the motors*/
		if (abs(sum.theta-current.theta) < 180)
		{
			vel.angular.z = RotCoef*0.02*(sum.theta-current.theta);
		}
		else if ((sum.theta-current.theta > 180) || (sum.theta-current.theta < -180))
		{
			vel.angular.z = RotCoef*-0.02*(sum.theta-current.theta);
		}
		ROS_INFO("Sum components 2: x:%f, y:%f, mag: %f, theta: %f",sum.x,sum.y, sum.mag, sum.theta);
		vel.linear.x = sum.mag;
		//limiting the linear speed to 0.75 m/s
		if (vel.linear.x >= MaxVel)
		{
			vel.linear.x = MaxVel;
		}
		if (vel.linear.x <= -1*MaxVel)
		{
			vel.linear.x = -1*MaxVel;
		}
		vel_pub.publish(vel);

	}
	else if (distanceToTarget < DistanceTh)
	{
		stop();
	}
}
void APFClass::poseCB(const nav_msgs::Odometry::ConstPtr &pose)
{
	current.x = pose->pose.pose.position.x;
	current.y = pose->pose.pose.position.y;
	current.theta = Rad2Deg * tf::getYaw(pose->pose.pose.orientation);
}
void stop(void)
{
	vel.linear.x = 0.0;
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;
	vel_pub.publish(vel);
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"mrs_apf");
	APFClass apfc;
	ros::spin();
	return 0;
}
