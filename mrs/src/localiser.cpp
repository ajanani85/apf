#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <sstream>

#define PI 3.14159265359
#define Rad2Deg 57.2957795
#define Deg2Rad 0.0174532925
#define PITCH 0.3522504892367

using namespace std;

struct point
{
	float x;
	float y;
	float theta;
};

int robot_id, requestNum = 0;
point cg, cl;
class LocaliserClass
{
public:
	LocaliserClass();
private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Subscriber pose_sub;
	
	void OdomCB(const nav_msgs::Odometry::ConstPtr &odom);
	void PoseCB(const nav_msgs::Odometry::ConstPtr &pose);
};

LocaliserClass::LocaliserClass()
{
	ros::NodeHandle n("~");
	n.getParam("robot_id",robot_id);
	ostringstream ss;
	ss << "robot_";
	ss << robot_id;
	string odom_pipename = ss.str() + "/odom";
	string pose_pipename = ss.str() + "/base_pose_ground_truth";

	odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_pipename, 10, &LocaliserClass::OdomCB, this);
  pose_sub = nh.subscribe<nav_msgs::Odometry>(pose_pipename, 10, &LocaliserClass::PoseCB, this);

}

void LocaliserClass::OdomCB(const nav_msgs::Odometry::ConstPtr &odom)
{
	float newPose[4] = {0.0,0.0,0.0,0.0}, newAngle;
	cl.x = odom->pose.pose.position.x;
	cl.y = odom->pose.pose.position.y;
	cl.theta = Rad2Deg * tf::getYaw(odom->pose.pose.orientation);


		//first sort out the angle
		newAngle = cg.theta;
		if (newAngle < -180)
		{
			newAngle = 360 + newAngle;
		}
		float R[2][2] = {{cos(newAngle*Deg2Rad), -1*sin(newAngle*Deg2Rad)},
										 {sin(newAngle*Deg2Rad), cos(newAngle*Deg2Rad)}};
		float x = cl.x*R[0][0] + cl.y*R[0][1];
		float y = cl.x*R[1][0] + cl.y*R[1][1];

		newPose[0] = x + cg.x;
		newPose[1] = y + cg.y;

		ROS_INFO("x: %f, y:%f",newPose[0],newPose[1]);
	

	
}

void LocaliserClass::PoseCB(const nav_msgs::Odometry::ConstPtr &pose)
{
	if (requestNum != 1)
	{
		cg.x = pose->pose.pose.position.x;
		cg.y = pose->pose.pose.position.y;
		cg.theta = Rad2Deg * tf::getYaw(pose->pose.pose.orientation);
		requestNum = 1;
	}
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"localiser");
	LocaliserClass lc;
	ros::spin();
	return 0;
}
