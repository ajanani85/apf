#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <sstream>

using namespace std;

ofstream my_file;
int robot_id = 0;
string file_name = "";
time_t now;
char* dt;
static char fileName[100];
double logged_x = 0.0, logged_y = 0.0;

class LogClass
{
 public:
		LogClass();
 private:
		ros::NodeHandle nh;
		ros::Subscriber pose_sub;
		void poseCallback(const nav_msgs::Odometry::ConstPtr &pose);
};

LogClass::LogClass()
{
	ros::NodeHandle n("~");
  n.getParam("robot_id",robot_id);
	
	ostringstream ss;
  
	ss << robot_id;

  string posepipe_name = "robot_"+ss.str() + "/base_pose_ground_truth";
  file_name = "//home//alireza//catkin_ws//src//mrs//log//r"+ss.str()+"_movement_%Y.%m.%d.csv";
	pose_sub = nh.subscribe<nav_msgs::Odometry>(posepipe_name,5,&LogClass::poseCallback,this);
	/*we overwrite on the previouse file*/
	//Creating the .csv log file
	now = time(0); //current data and time based on current system
	dt = ctime(&now); 
	strftime(fileName, sizeof(fileName), file_name.c_str(), localtime(&now));
	my_file.open(fileName, ios_base::app);//ios_base::app appends data to the current content
	my_file << "Sequence, time(s), pose_x(m), pose_y(m)"<<endl;	
	my_file.close();
}

void LogClass::poseCallback(const nav_msgs::Odometry::ConstPtr &pose)
{
	
	if (logged_x != pose->pose.pose.position.x || logged_y != pose->pose.pose.position.y)
  {
	  static char currentTime[20];	
	  now = time(0); //current data and time based on current system
	  dt = ctime(&now); 
	  strftime(fileName, sizeof(fileName), file_name.c_str(), localtime(&now));
	  strftime(currentTime, sizeof(currentTime), "%X", localtime(&now));
	  my_file.open(fileName, ios_base::app);//ios_base::app appends data to the current content
	  my_file <<pose->header.seq<<", "<< pose->header.stamp<<", "<<pose->pose.pose.position.x<<", "<<pose->pose.pose.position.y<<endl;
	  my_file.close();
	  logged_x = pose->pose.pose.position.x;
	  logged_y = pose->pose.pose.position.y;
  }
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"logger");
	LogClass lg;
	ros::spin();
	return 0;
}

