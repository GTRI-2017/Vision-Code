/*
	This file was used to test how to subscribe to the camera feed from the drone.
	NOTE: Not the actual detection process
	

*/ 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  bool img_received = false;
  //image_transport allows you to subscribe to compressed image streams
  cv::Mat img;


  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, //subscirbed topic has been changed
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW); 
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	//converts ROS image to CVImage type and makes a mutable(changeable) copy of image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
       //std::cout << img;
       img_received = true;
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    img_received = true;

    // Draw an example circle on the video stream
    //if (img.rows > 60 && img.cols > 60)
      cv::circle(img, cv::Point(100, 100), 100, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void run()
  {
    ros::Rate loop_rate(10); // 10 Hz
  	while(ros::ok()) {
  	
  		std::cout << "while loop entered" << endl;
  		
  		if (img_received) {
  		std::cout << "if loop entered " << endl;
  	
  	

  	vector<int> markerIds;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    
  
    Ptr<aruco::Dictionary> markerDictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0)); //0 = DICT_4x4_50
    ROS_INFO ("AA");
    
	 aruco::detectMarkers(img, markerDictionary, markerCorners, markerIds);
	ROS_INFO ("BB");
  	
   			        
  	  ros::spinOnce();
      loop_rate.sleep();
     }
    
  }


}
};

/*void run(Mat& frame){
 
 		 vector<int> markerIds;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    Ptr<aruco::Dictionary> markerDictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0)); //0 = DICT_4x4_50

	 aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);

}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    std::cout << "Run started" << endl;
    ic.run();
    std::cout << "Run Completed" << endl;
	  return 0; 
  }
  


