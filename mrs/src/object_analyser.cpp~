#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>

#define PI 3.14159265359
#define Rad2Deg 57.2957795
#define Deg2Rad 0.0174532925
#define PITCH 0.3522504892367
#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
using namespace std;

int robot_id;
float p_dx[30], p_dy[30], speed;

struct point
{
	float x;
	float y;
	float theta;
};

point p_cg;
struct laser_object
{
	int numberOfBeams;
	int low_beam_number;
	int high_beam_number;
	float x;
	float y;
	bool isItMoving;
};
point cg;
laser_object pre_obj[30];
class LaserClass
{
public:
	LaserClass();
private: 
	ros::NodeHandle nh;
	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;


	void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan);
	void poseCB(const nav_msgs::Odometry::ConstPtr &pose);
};

LaserClass::LaserClass()
{
	ros::NodeHandle n("~");
	n.getParam("robot_id",robot_id);

	ostringstream ss;
	ss << "robot_";
	ss << robot_id;

	string laser_pipename = ss.str() + "/base_scan";
	string pose_pipename = ss.str() + "/base_pose_ground_truth";
	laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_pipename, 10, &LaserClass::laserCB, this);
	pose_sub = nh.subscribe<nav_msgs::Odometry>(pose_pipename,10,&LaserClass::poseCB, this);


	for (int i=0; i<=29; i++)
	{
		pre_obj[i].numberOfBeams = 0;						
		pre_obj[i].low_beam_number = 0;
		pre_obj[i].high_beam_number = 0;
		pre_obj[i].x = 0.0;
		pre_obj[i].y = 0.0;
		pre_obj[i].isItMoving = false;
	}


	p_cg.x = 0.0;
	p_cg.y = 0.0;
	p_cg.theta = 0.0;
}
void LaserClass::laserCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	float ang = (90.0-cg.theta)*Deg2Rad;
	float R[2][2] = {{cos(ang),-1*sin(ang)}, {sin(ang),cos(ang)}};
	int prev_beam_index= 0, beam_index = 0, object_count = 0, numOfBeams =0;
	laser_object obj[30];
	int tNumObj =0;
	//initialise the array of objects
	for (int i=0; i<=29; i++)
	{
		obj[i].numberOfBeams = 0;						
		obj[i].low_beam_number = 0;
		obj[i].high_beam_number = 0;
		obj[i].x = 0.0;
		obj[i].y = 0.0;
		obj[i].isItMoving = false;
	}
	point loc[512], global[512];
	ROS_INFO("------------------------------");
	//ROS_INFO("cg.theta: %f,ang: %f",cg.theta,ang*Rad2Deg);
	//First read beams and their angles
	for (int i=0; i<=511; i++)
	{
		if (scan->ranges[i] < 4.5)
		{
			//Finding the global position of each beam
			beam_index = i;
			float x = scan->ranges[i]*cos(i*PITCH*Deg2Rad);
			float y = scan->ranges[i]*sin(i*PITCH*Deg2Rad);
				
			//ROS_INFO("detected object local[%d], x:%f, y:%f",i,x, y);	
	
			float x1 = x*cos(ang) + y*sin(ang);
			float y1 = -x*sin(ang) + y*cos(ang);

			global[i].x = x1 + cg.x;
			global[i].y = y1 + cg.y;	
			
			//ROS_INFO("detected object global[%d], x:%f, y:%f",i,global[i].x,global[i].y);
			
			//Finding the total number of objects: this is for finding the structure array size
			/*if (i == 0)
			{
				tNumObj++;
				numOfBeams++;
				prev_beam_index = 0;
			}
			else if (i > 0 && beam_index > prev_beam_index + 1)
			{
				tNumObj++;
				numOfBeams++;
				prev_beam_index = beam_index;
			}
			else if (i >0 && beam_index == prev_beam_index + 1)
			{
				numOfBeams++;
				prev_beam_index = beam_index;
			}*/
			
		}
		else
		{
	
		}
		
	}
	//ROS_INFO("total number of detected objects: %d, and number of beams associated with it: %d",object_count,numOfBeams);
	//Allocating size to the dynamic array of laser_objects
	//obj = new laser_object[object_count];	
	beam_index = 0;
	prev_beam_index = 0;
	object_count = 0;
	numOfBeams =0;
	float x_avg = 0.0, y_avg = 0.0;
	int low_beam = 0, high_beam = 0;


	for (int i=0; i<=511; i++)
	{
		if (scan->ranges[i] < 4.5)
		{
			beam_index = i;
			if (i == 0)
			{
				object_count++;
				numOfBeams++;
				prev_beam_index = 0;
				x_avg += global[i].x;
				y_avg += global[i].y;
				low_beam =0;
			}

			//that is a new object entry
			if (i > 0 && beam_index > prev_beam_index + 1)
			{
				object_count++;
				numOfBeams = 1;
				x_avg = global[i].x;
				y_avg = global[i].y;
				/*obj[object_count].numberOfBeams = numOfBeams;						
				obj[object_count].low_beam_number = low_beam;
				obj[object_count].high_beam_number = high_beam;
				obj[object_count].x = x_avg/numOfBeams;
				obj[object_count].y = y_avg/numOfBeams;
				ROS_INFO("obj[%d]: nBeam:%d low:%d, high:%d, x:%f, y:%f",object_count,obj[object_count].numberOfBeams,obj[object_count].low_beam_number,
				obj[object_count].high_beam_number, obj[object_count].x,obj[object_count].y);*/
				prev_beam_index = beam_index;
				low_beam = beam_index;
			}
			if (i > 0 && beam_index == prev_beam_index + 1)
			{
				numOfBeams++;
				prev_beam_index = beam_index;
				x_avg += global[i].x;
				y_avg += global[i].y;
				high_beam = beam_index;
			}
			//an object that is detected at 511 has no end
			if (i == 511 && scan->ranges[i-1] < 4.5)
			{
				//ROS_INFO("HELLO");
				obj[object_count].numberOfBeams = numOfBeams;						
				obj[object_count].low_beam_number = low_beam;
				obj[object_count].high_beam_number = high_beam;
				obj[object_count].x = x_avg/numOfBeams;
				obj[object_count].y = y_avg/numOfBeams;
				//ROS_INFO("obj[%d] out of %d: nBeam:%d low:%d, high:%d, x:%f, y:%f",object_count, tNumObj,obj[object_count].numberOfBeams,obj[object_count].low_beam_number, obj[object_count].high_beam_number, obj[object_count].x,obj[object_count].y);
	
				/*obj[object_count-1].numberOfBeams = numOfBeams;						
				obj[object_count-1].low_beam_number = low_beam;
				obj[object_count-1].high_beam_number = high_beam;
				obj[object_count-1].x = x_avg/numOfBeams;
				obj[object_count-1].y = y_avg/numOfBeams;
				ROS_INFO("obj[%d]: nBeam:%d low:%d, high:%d, x:%f, y:%f",object_count-1,obj[object_count-1].numberOfBeams,obj[object_count-1].low_beam_number, obj[object_count-1].high_beam_number, obj[object_count-1].x,obj[object_count-1].y);*/
			}
			
		}
		//you reached the end of an object
		else if (i > 0 && scan->ranges[i] >= 4.5 && scan->ranges[i-1] < 4.5)
		{
			obj[object_count].numberOfBeams = numOfBeams;						
			obj[object_count].low_beam_number = low_beam;
			obj[object_count].high_beam_number = high_beam;
			obj[object_count].x = x_avg/numOfBeams;
			obj[object_count].y = y_avg/numOfBeams;
			//ROS_INFO("obj[%d] out of %d: nBeam:%d low:%d, high:%d, x:%f, y:%f",object_count, tNumObj,obj[object_count].numberOfBeams,obj[object_count].low_beam_number, obj[object_count].high_beam_number, obj[object_count].x,obj[object_count].y);
		}
		
	}
 //Now that we have all the required information, we can figure out if the detected objects are moving or not.
 //First, lets consider that the robot is not moving and other items are moving.
 	for (int i=0; i<30; i++)
 	{
		if (obj[i].numberOfBeams > 0)
		{
			if (pre_obj[i].numberOfBeams > 0)
			{
				float dx = obj[i].x - pre_obj[i].x;
				float dy = obj[i].y - pre_obj[i].y;

				//ROS_INFO("obj[%d], ddf: %f, rdf: %f", i,dx-p_dx[i], sqrt(pow(cg.x-p_cg.x,2) + pow(cg.y-p_cg.y,2)));				                                    
		    //ROS_INFO("obj[%d], number of Beam:%d, change in c_x:%f, change in p_x:%f, diff:%f", i, obj[i].numberOfBeams, dx-cg.x, p_dx[i]-p_cg.x,abs(dx-cg.x - p_dx[i] +p_cg.x));	
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             					//ROS_INFO("obj[%d]: changes in x:%f, y:%f", i,dx,dy);
				//ROS_INFO("obj[%d], nBeam:%d, nBeam_old:%d, diff:%d", i,obj[i].numberOfBeams,pre_obj[i].numberOfBeams,obj[i].numberOfBeams-pre_obj[i].numberOfBeams);
				if ((dx != 0.0 || dy!= 0.0) && (abs(p_dx[i]) != abs(dx) && abs(p_dy[i]) != abs(dy)))
				{
					//ROS_INFO("obj[%d]: is alive", i);
					p_dx[i] = dx;
					p_dy[i] = dy;
				}
				else if ((dx == 0.0 && dy== 0.0) || (abs(p_dx[i]) == abs(dx) && abs(p_dy[i]) == abs(dy)))
				{
					
					//ROS_INFO("obj[%d]: is dead", i);
					
					
					p_dx[i] = dx;
					p_dy[i] = dy;
				}

				pre_obj[i].numberOfBeams = obj[i].numberOfBeams;						
				pre_obj[i].low_beam_number = obj[i].low_beam_number;
				pre_obj[i].high_beam_number = obj[i].high_beam_number;
				pre_obj[i].x = obj[i].x;
				pre_obj[i].y = obj[i].y;
				pre_obj[i].isItMoving = obj[i].isItMoving;
				
				p_cg.x = cg.x;
				p_cg.y = cg.y;
				p_cg.theta = cg.theta;
			}			
			if (pre_obj[i].numberOfBeams == 0)
			{
				pre_obj[i].numberOfBeams = obj[i].numberOfBeams;						
				pre_obj[i].low_beam_number = obj[i].low_beam_number;
				pre_obj[i].high_beam_number = obj[i].high_beam_number;
				pre_obj[i].x = obj[i].x;
				pre_obj[i].y = obj[i].y;
				pre_obj[i].isItMoving = obj[i].isItMoving;
			}
			
		}
	}  	

	//releasing the dynamic memory
	//delete[] obj;
}
void LaserClass::poseCB(const nav_msgs::Odometry::ConstPtr &pose)
{
	cg.x = pose->pose.pose.position.x;
	cg.y = pose->pose.pose.position.y;
	cg.theta = Rad2Deg*tf::getYaw(pose->pose.pose.orientation);

	speed = pose->twist.twist.linear.x; //sqrt(pow(pose->twist.twist.linear.x,2) + pow(pose->twist.twist.linear.y,2));
	
}
int main(int argc, char** argv)
{
 ros::init(argc,argv,"object_analyser");
 LaserClass lc;
 ros::spin();
 return 0;
}
