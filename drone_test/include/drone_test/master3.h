/********************************************
								GTRI ATAS
    					Header File master3.h
							Created: 6/27/17
          Updated: 7/18/17 9:01 AM
********************************************/

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <unistd.h>


class Master
	{
	public:
  	ros::NodeHandle node1;
    ros::Publisher pub_twist;
    ros::Subscriber nav_sub;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Subscriber pos_sub;
  
    double xgo;
  	double ygo;
  	double tc=0;
  	double centered;
  
  	double vx_, vy_, vz_;
  	double height;
  
  	//set up the system we use to cmd x,y,z coords to drone (linear & angular)
    	geometry_msgs::Twist stop_msg;
      geometry_msgs::Twist forward_msg;
      geometry_msgs::Twist turn_msg;
      geometry_msgs::Twist explore_msg;
    	geometry_msgs::Twist down_msg;
  		geometry_msgs::Twist centering_msg;
  		std_msgs::Empty takeoff_msg;
  		std_msgs::Empty land_msg;

    Master();
  	//void nav_callback(const ardrone_autonomy::Navdata& msg_in);
    void stopf();
  	void corner();
    void octagonthing();
    void explore();
    void center(); // was approach
    void lower();
    void roomba();
    void gotaway();
    void run();
  		
  };
