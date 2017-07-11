/*
GTRI ATAS
Find Tag
Created 6/27/17
Updated 6/28/17 :: 1:00 pm
*/

#include <ros/ros.h> 
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

int tc; //tag count
int tt; //tag type 

std_msgs::Empty emp_msg;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
//this is called when a new message arrives on the nav topic.
	{
	  //Take in state of ardrone
  tc = msg_in.tags_count;
	tt = msg_in.tags_count;
	}


int main(int argc, char **argv)
	{
  ROS_INFO("Starting search pattern node");
	ros::init(argc, argv,"ardrone_search"); //NOTE: "ardrone_search" is name of node
 	ros::NodeHandle node;
	ros::Rate loop_rate(10);
  
  ros::Subscriber nav_sub;
   
  	nav_sub = node.subscribe("/ardrone/navdata", 10, nav_callback); // 1->10
  	
  	while (ros::ok)  
    		{
        
        
      	ROS_INFO("TAG Count = %i", tc);
		while (tc < 1) // if the tag is spotted, will exit this loop
			{
			
			ROS_INFO("TAG Count = %i", tc);
			ros::spinOnce();
			loop_rate.sleep();
      		          		}
		//Put what it does after it finds a tag here
		ROS_INFO("-------------------------Tag has been detected-----------------------------------");
    		}
    		
    	//ros::spinOnce();
    	//	loop_rate.sleep();
	}
