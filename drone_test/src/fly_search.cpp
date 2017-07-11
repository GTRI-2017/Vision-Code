/*
GTRI ATAS
Search and Chase Tag
Created 6/27/17
Updated 6/2/17 :: 2:54 AM
*/

#include <ros/ros.h> 
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <unistd.h>

double vx_, vy_, vz_;
int newa;
int tc;
int altitude;

/*
char flightStartChars [] = { 'S', 'T' , ' A' , 'R', 'T', 'I','N','G','','D','R','O','N','E' }; //TYPING FUNCTION
double seconds = .25; //variable for the weight time for the type() function 
*/

//set up the system we use to cmd x,y,z coords to drone (linear & angular)
geometry_msgs::Twist corner1_msg;
geometry_msgs::Twist corner2_msg;
geometry_msgs::Twist corner3_msg;
geometry_msgs::Twist loop1_msg;
geometry_msgs::Twist loop2_msg;
geometry_msgs::Twist loop3_msg;
geometry_msgs::Twist loop4_msg;
std_msgs::Empty emp_msg;

/*
void type (char phrase)
	{ //function that prints out a phrase letter by letter
  for (int i = 0; i < phrase.sizeof(phrase); i++)
  	{
  	ROS_INFO("%i", phrase[i]);
    sleep(seconds);
  	}
	}
*/


void nav_callback(const ardrone_autonomy::Navdata& msg_in)
//this is called when a new message arrives on the nav topic.
	{
	ROS_INFO("Getting sensor reading");
  //Take in state of ardrone
  vx_=msg_in.vx*0.001;
	vy_=msg_in.vy*0.001;
	vz_=msg_in.vz*0.001;
  newa = msg_in.altd;
  tc = msg_in.tags_count;
	}


//Function for finding the average altitude of the drone because the ultrasonic sensor is inaccurate 
double averageAltd ()
  { //function that can average the altitude b/c the ultrasonic sensor is booty 
  double sum = 0; //defines sum as altitude (say, 1.2m)
  
  for (double i = 0; i < 10; i++ )
    { //will loop 10 times (if you count 0)
    sum = sum + newa; //adds new altd measurement to existing sum variable
    ROS_INFO("function loop %i", i);
    }
  altitude = sum/10;
  return altitude ; //return altitude (in meters)
}

int main(int argc, char **argv)
	{

	ROS_INFO("Starting PARROT AR DRONE V1.0");
  ROS_INFO("Starting search pattern node");
	ros::init(argc, argv,"ardrone_search"); //NOTE: "ardrone_search" is name of node
 	ros::NodeHandle node;
	ros::Rate loop_rate(50);
  
  ros::Publisher pub_twist;
  ros::Subscriber nav_sub;
  ros::Publisher pub_empty; //takeoff
  
  
	ROS_INFO("DECLARING FLIGHT VECTORS");
////////////////////////////////////////////////////////////////////////
  /*SEARCH VECTORS*/ //Corners set to 0.25 v and Straights 0.5 v
/////////////////////////////////////////////////////////////////////
  /*Setup corner1: go forward*/
  	corner1_msg.linear.x=0.333; // forward
  	corner1_msg.linear.y=0.0; // to the side
  	corner1_msg.linear.z=0.0; // up
  	corner1_msg.angular.x=0.0; // not to use
  	corner1_msg.angular.y=0.0; // not to use
  	corner1_msg.angular.z=0.0; //spin
  /*Setup corner2: turn*/
  	corner2_msg.linear.x=0.0; // forward
    corner2_msg.linear.y=0.0; // to the side
    corner2_msg.linear.z=0.0; // up
    corner2_msg.angular.x=0.0; // not to use
    corner2_msg.angular.y=0.0; // not to use
    corner2_msg.angular.z=0.523; //spin
  /*Setup corner3: go to corner*/
  	corner3_msg.linear.x=0.167; // forward
    corner3_msg.linear.y=0.0; // to the side
    corner3_msg.linear.z=0.0; // up
    corner3_msg.angular.x=0.0; // not to use
    corner3_msg.angular.y=0.0; // not to use
    corner3_msg.angular.z=0.0; //spin
  /*Setup loop1: turn*/
  	loop1_msg.linear.x=0.0; // forward
    loop1_msg.linear.y=0.0; // to the side
    loop1_msg.linear.z=0.0; // up
    loop1_msg.angular.x=0.0; // not to use
    loop1_msg.angular.y=0.0; // not to use
    loop1_msg.angular.z=0.743; //spin
  /*Setup loop2: forward*/
  	loop2_msg.linear.x=0.653; // forward
    loop2_msg.linear.y=0.0; // to the side
    loop2_msg.linear.z=0.0; // up
    loop2_msg.angular.x=0.0; // not to use
    loop2_msg.angular.y=0.0; // not to use
    loop2_msg.angular.z=0.0; //spin
  /*Setup loop3: turn*/
  	loop3_msg.linear.x=0.0; // forward
    loop3_msg.linear.y=0.0; // to the side
    loop3_msg.linear.z=0.0; // up
    loop3_msg.angular.x=0.0; // not to use
    loop3_msg.angular.y=0.0; // not to use
    loop3_msg.angular.z=0.743; //spin
  /*Setup loop4: forward*/
  	loop4_msg.linear.x=0.333; // forward
    loop4_msg.linear.y=0.0; // to the side
    loop4_msg.linear.z=0.0; // up
    loop4_msg.angular.x=0.0; // not to use
    loop4_msg.angular.y=0.0; // not to use
    loop4_msg.angular.z=0.0; //spin
//////////////////////////////////////////////////
  
  	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
  	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 10); //takeoff
  
  	while (ros::ok) //While ROS is emotionally stable (she's going thru tough times)
    		{
    		loop_rate.sleep();
    		double time_start=(double)ros::Time::now().toSec();
        while ((double)ros::Time::now().toSec()< time_start+5.0 && altitude < 150)
        	{
          ROS_INFO("Taking Off");
          pub_empty.publish(emp_msg); /* launches the drone */
          ros::spinOnce();
          loop_rate.sleep();
      		ROS_INFO("Altitude is %i", altitude);
    			}
            
    		ROS_INFO("Beginning flight pattern");
				while (tc < 1) // if the tag is spotted, will exit this loop
						{
      			ROS_INFO("Searching ...");
      			
      			double turnt = (double)ros::Time::now().toSec(); // Time for turns is set to run for 1 second
       		 	while ((double)ros::Time::now().toSec()< (turnt + 3))
							{
							ROS_INFO("In turnt 1");
							pub_twist.publish(corner1_msg);
							ros::spinOnce();
        			loop_rate.sleep();
							}
						while ((double)ros::Time::now().toSec()< (turnt + 6))
							{
							ROS_INFO("In turnt 2");
							pub_twist.publish(corner2_msg);
							ros::spinOnce();
        			loop_rate.sleep();
							}
						while ((double)ros::Time::now().toSec()< (turnt + 9))
							{
							ROS_INFO("In turnt 2");
							pub_twist.publish(corner3_msg);
              ros::spinOnce();
        			loop_rate.sleep();
							}
						while (1) // search loop
							{
          		double stime = (double)ros::Time::now().toSec(); // Straights are set to run for 5 sec
            	while ((double)ros::Time::now().toSec()< stime+3.0)//2
								{
								pub_twist.publish(loop1_msg);
								ros::spinOnce();
        				loop_rate.sleep();
                }
              while ((double)ros::Time::now().toSec()< stime+6.0)
								{
								pub_twist.publish(loop2_msg);
								ros::spinOnce();
        				loop_rate.sleep();
								}
							while ((double)ros::Time::now().toSec()< stime+9.0)
								{
								pub_twist.publish(loop3_msg);
								ros::spinOnce();
        				loop_rate.sleep();
								}
							while ((double)ros::Time::now().toSec()< stime+12.0)
								{
                pub_twist.publish(loop4_msg);
								ros::spinOnce();
        				loop_rate.sleep();
								}
							}
            }
  	//Put what it does after it finds a tag here
    		}
	}

 
