#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//#include <ros/vision_opencv.h>

// header files


using std::cout;
using std::endl;

class ClassName
{
public:
     ClassName()
	  {
	       // this constructor gets called once

	       // publish to topics of relevance
	     
	     pub_ = n_.advertise<std_msgs::String>("chatter", 1);
					
	       // subscribers
	       sub_ = n_.subscribe("chatter", 1, &ClassName::cb_fun, this);
	  }
public:
     // callback functions
    void cb_fun(const std_msgs::String::ConstPtr& msg)
	  {
	  	ROS_INFO("I heard: [%s]", msg->data.c_str());
	  
	  }

     // user-defined functions

     void run()
     	  {
	       ros::Rate loop_rate(10); // 10 Hz

	       while (ros::ok()) {
	       
	       std_msgs::String msg;
	       
     			std::stringstream ss;
				 ss << "hello world! " << count;
				 msg.data = ss.str();
		//    ROS_INFO("%s", msg.data.c_str());
		    pub_.publish(msg);
		    count++;

		    ros::spinOnce();
		    loop_rate.sleep();
	       }
     	  }

private:
		 ros::NodeHandle n_;
     ros::Publisher pub_;
     ros::Subscriber sub_;
     // global variables
     
     int count = 0;
};

// -------------------------------------

int main(int argc, char **argv)
{
     // initialize ROS and node
     ros::init(argc, argv, "templatetest");
     //ros::init(argc, argv, "listener");
     ClassName class_instance;
     class_instance.run();
     ros::spin();
     return 0;
}
