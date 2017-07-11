/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This program launches the AR Drone. 
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
std_msgs::Empty emp_msg;	
using namespace std;
int main(int argc, char** argv)
{

	ROS_INFO("Flying ARdrone");
	ros::init(argc, argv,"ARDrone_test");
	ros::NodeHandle node;
  ros::Rate loop_rate(50);
	ros::Publisher pub_empty;
	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 10); /* Message queue length is just 1 */
	
 	while (ros::ok()) 
 				{
 				//ros::spinOnce();
 				//pub_empty.publish(emp_msg);
 				//loop_rate.sleep();
				double time_start=ros::Time::now().toSec();
				cout << "time: " << ros::Time::now().toSec() << endl;
				cout << "time_start Before Loop: " << time_start << endl;
				while ((double)ros::Time::now().toSec()< time_start+5.0) /* Send command for five seconds*/
					{
					ROS_INFO("Taking Off"); 
					cout << "time: " << ros::Time::now().toSec() << endl;
					pub_empty.publish(emp_msg); /* launches the drone */
					ros::spinOnce();
					loop_rate.sleep();
					cout << "time_start: " << time_start << endl;
					cout << "time: " << ros::Time::now().toSec() << endl;
					}//time loop
				
				ROS_INFO("ARdrone launched");
				
				exit(0);
				}//ros::ok loop

}//main
