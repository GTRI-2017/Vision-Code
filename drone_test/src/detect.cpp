/***************************************************************************
		GTRI ATAS SUMMER INTERNSHIP 2017
	             VISION PROCESSING CODE
	            DEVELOPER: ALLEN AYALA
*****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h> //drone video feed
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <image_transport/image_transport.h>
#include <aruco/marker.h>

  
#include <sstream>
#include <iostream>
#include <fstream>
//NOTE:: In source file, if function is not static, it's an instance function
using namespace std;
using std::cout;
using std::endl;

const float calibrationSquareDimension = .0270f; //meters (approx.)
const float arucoSquareDimension = .132; //needs to be standard 0.032f
const cv::Size chessboardDimensions = cv::Size(9,6);
const int videoPort = 0;

//center camera frame, changes between drones
//Camera Dimensions (x & y):
// P1B: 144 x 174  P2B: 640 x 360

//constants containing center camera coordinates of each drone's bottom camera
const float x_P1B = 72;
const float y_P1B = 87;
const float x_P2B = 320;
const float y_P2B = 180;

//assigning constant used in code based on drone being tested
const float x_center = x_P2B;
const float y_center = y_P2B;
const float error = 25; 

//constants that make sure marker centroid falls within boundary region
const float x_ideal_minimum = x_center - error;
const float x_ideal_maximum = x_center + error;
const float y_ideal_lower = y_center + error;
const float y_ideal_higher = y_center - error;

//movements are in reference to the bottom camera; tell where to move the marker
bool move_right = false;
bool move_left = false;
bool move_up = false;
bool move_down = false;
bool tagDetected = false; 
bool land = false; // tells whether or not the drone is ready to be landed

// NOTE: corners (in calibration) refer to the intersection points of the checker board squares

void createKnownBoardPosition(cv::Size boardSize, float squareEdgeLength, vector<cv::Point3f>& corners){
//places 3D world coordinates of each corner into a vector
// used for camera calibration 
    for(int i = 0; i < boardSize.height; i++){

        for(int j=0; j < boardSize.width; j++){
            corners.push_back(cv::Point3f(j  * squareEdgeLength, i * squareEdgeLength, 0.0f)); //z=0 in flat planes
        }
    }
}

//                                              vector of vectors, passed by reference
void getChesssboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f> >& allFoundCorners, bool showResults = false){
//obtains the 3D world coordinate of each corner from camera
// used for camera calibration 
    for(vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++){

        vector<cv::Point2f> pointBuf; //temporary vector
        bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //bitwise "or" operator

        if(found){

            allFoundCorners.push_back(pointBuf);
        }

        if(showResults){ //draws corners onto image

            drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
            imshow("Looking for Corners", *iter); //shows camera feed with drawn calibration points
            cv::waitKey(0);
        }

    }

}

void compareToCenter(float x, float y ){
// interprets 2D marker centroid coordinates (in reference to the camera frame) and compares it to the camera center
    if( (x >= x_ideal_minimum && x <= x_ideal_maximum) && (y <= y_ideal_lower && y >= y_ideal_higher)) {
        land = true;
        cout << " -------------------------In Center -----------------------" << endl;
    }else{
        land = false;
        }
    
   if(x > 0 && y > 0 ){
	//filters out error readings
            tagDetected = true;
            if(x < x_ideal_minimum){
                move_right = true;
                cout << "MOVE RIGHT!!!" << endl;
            }else{
                move_right = false;
            }

            if(x > x_ideal_maximum){
                move_left = true;
                cout << "MOVE LEFT!!!" << endl;
            }else{
                move_left = false;
                }

            if(y < y_ideal_higher){
                move_down = true;
                cout << "MOVE DOWN!!!" << endl;
            }else{
                move_down = false;}

            if(y > y_ideal_lower){
                move_up = true;
                cout << "MOVE UP!!!" << endl;
            }else{
                move_up = false;
            }
            cout << "X Ideal Min: " << x_ideal_minimum << endl;
            
            cout << "Difference in X: " << (x- x_center) << endl;
            cout << "Difference in Y: " << (y- y_center) << endl;
   }else{
       tagDetected = false;
   }
        
}

int startWebcamMonitoring(const cv::Mat& cameraMatrix, const cv::Mat& distanceCoefficients, float arucoSquareDimension, cv::Mat& frame){
	// ArUco detection and pose edtimation performed live on drone camera feed     

    vector<int> markerIds;
	
    vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters parameters;

    cv::Ptr<cv::aruco::Dictionary> markerDictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(0)); //0 = DICT_4x4_50

    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    vector<cv::Vec3d> rotationVectors, translationVectors;
     //vector<> _objPoints; //new
//     vector<> _objPoints;
    while(true){


        cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix,
                          distanceCoefficients, rotationVectors, translationVectors);
       // prints out X and y coordinates of markerCorners
       float x_sum = 0;
       float y_sum = 0;
       float x_average, y_average;

        cout << "-------------------Coordinates ---------------------------" << endl; 
        for(int i=0; i<markerCorners.size(); i++) // Ensure that you don't access any elements that don't exist
            for(int p=0; p<markerCorners[i].size(); p++) {
			//finds average of marker corner coordinates             
            x_sum  += markerCorners[i][p].x;
            y_sum  += markerCorners[i][p].y;
            }
            x_average = x_sum/4;
            y_average = y_sum/4;
			
			//NOTE: taking the average of the 2D coordinated of the corners results in the coordinates for the centroid

            cout << "\n Mean X: " << x_average << " Mean Y: " << y_average << endl; //prints out 2D coordinate of ArUco marker centroid 
            compareToCenter(x_average, y_average); //performs comparison with marker centroid
               
        for(int i = 0;  i < markerIds.size(); i ++){

            cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);

        }

        imshow("Webcam",  frame);
        if (cv::waitKey(30) >= 0) break; //if nothing is detected within 30 seconds, program stops
    }
    return 1;
}



void cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients){
	// performs calibration algorithms on saved images
	vector<vector<cv::Point2f> > checkerboardImageSpacePoints;
    getChesssboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    vector<vector<cv::Point3f> > worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<cv::Mat> rVectors, tVectors; //rotation and translation vectors
    distanceCoefficients = cv::Mat::zeros(8,1, CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix,
                    distanceCoefficients, rVectors, tVectors );
}




bool saveCameraCalibration(string name, cv::Mat cameraMatrix, cv::Mat distanceCoefficients){
    //save camera properties to a file
    ofstream outStream(name.c_str()); //new: originally ..outStream(name); for C++11 but modified is for C++03 
    if (outStream){

        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;
		//saves data into the file
        outStream << rows << endl; 
        outStream << columns << endl;

        for(int r =0; r < rows; r++){

            for(int c = 0; c < columns; c++){

                double value = cameraMatrix.at<double>(r,c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;
		//saves data into the file
        outStream << rows << endl; 
        outStream << columns << endl;

        for(int r =0; r < rows; r++){

            for(int c = 0; c < columns; c++){

                double value = distanceCoefficients.at<double>(r,c);
                outStream << value << endl;
            }
        }

        outStream.close();
        return true;
    }

    return false;
}

bool loadCameraCalibration(string name, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients){
	//loads proper calibration paramters for specific camera
    ifstream inStream(name.c_str()); //for C++03
    if (inStream){

        uint64_t rows, columns;
		//reads values from the file
        inStream >> rows; 
        inStream >> columns;

        cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

 
        //iterate and reads over cameraMatrix values
        for(int r = 0; r < rows; r++){

            for(int c= 0; c <columns; c++){

                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r,c) = read;

               // cout << cameraMatrix.at<double>(r,c) << "\n";
            }
        }
       //grab all the distanceCoefficients

        inStream >> rows;
        inStream >> columns;

        distanceCoefficients = cv::Mat(cv::Size(columns, rows), CV_64F);

       // cout << "Distance Coefficients" << endl;
      //  cout << "Rows: " << rows << endl;
       // cout << "Cols: " << columns << endl;

        for(int r = 0; r < rows; r++){

            for(int c= 0; c < columns; c++){

       
                double read = 0.0f;
                inStream >> read;
                distanceCoefficients.at<double>(r,c) = read;
				}
           }
        inStream.close();
        return true;
        }

    return false;
}

 vector<cv::Mat> savedImages;

void cameraCalibrationProcess(cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients, cv::Mat& frame){ 
//gets calibration images from live camera feed   
    cv::Mat drawToFrame;

    vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;

    int framesPerSecond = 20;

    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE); // draws an OpencCv window 

        vector<cv::Vec2f> foundPoints;
        bool found = false; 
			
        if (frame.empty()){
        ROS_INFO("MATRIX IS EMPTY!");
        }else{
            found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                
        }
        
        frame.copyTo(drawToFrame);
       
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
     
        if(found){
            
            imshow("Webcam", drawToFrame);
    }else{
           
            imshow("Webcam", frame);  
    }
        char character = cv::waitKey(1000 / framesPerSecond); //capture typed chars

        switch(character){
		//checks input for saving images, performing calibration algorithm, or exiting the calibration        
        case ' ':
            //SPACE - saving image
            ROS_INFO("SPACBAR ----------------------------------------SPACEBAR----");
            if(found){
                cv::Mat temp;
                frame.copyTo(temp);
                savedImages.push_back(temp);
                std::cout << "SPACEBAR pressed - Image: " << (savedImages.size())<< endl;
            }
            break;
        case 13:
            //ENTER - start calibration
            ROS_INFO( "ENTER Pressed ----------------------------- ENTER--");
            
            if (savedImages.size() > 15){

                std::cout << "Starting calibration..." << endl;
                cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                saveCameraCalibration("Drone2B", cameraMatrix, distanceCoefficients);
                std::cout << "I done finished calibration" << endl;
            }
            break;
        case 27:
            //ESCAPE - exit
			std::cout << "calibration finished" << endl;
            return; // 0;
            break;

        }
 }
  
 bool img_received_ = false;  //image processing
 bool video_received_ = false; //video feed
static const std::string OPENCV_WINDOW = "Image window";
	cv::Mat img;

     

class ImageConverter
{
public:
 
  geometry_msgs::Twist pos_msg; // msg to publish tag identified and able to land

  ros::NodeHandle nh_;
  ros::Publisher pub_pos = nh_.advertise<geometry_msgs::Twist>("/ardrone/tag_pos", 1); //topic used to send tag info
   
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //image_transport allows you to subscribe to compressed image streams
    ImageConverter()
    : it_(nh_)
  {
    cout << "instance created" << endl;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 100, //subscirbed topic has been changed
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 100);

    cv::namedWindow(OPENCV_WINDOW);

    //If code below not within a constructor, error occurs: "pos_msg" does nor name type 
    pos_msg.linear.x = 0.0; // x direction for tag
    pos_msg.linear.y = 0.0; // y direction for tag
    pos_msg.linear.z = 0.0; //tag count : 0 or 1
    pos_msg.angular.x = 0.0; // tag is centered? able to land?
    pos_msg.angular.y = 0.0; //not used
    pos_msg.angular.z = 0.0; //not used

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW); 
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
  
  	//std::cout << "Call back called " << endl;
  	//converts ROS image to CVImage type and makes a mutable(changeable) copy of image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
       
        img_received_ = true;
        video_received_ = true;
       
        int rows = img.rows;
        int cols = img.cols;
       
        cv::Size s = img.size();
        rows = s.height;//240 for P1 and 360 P2
        cols = s.width;//320 for P1 and 640 for P2
      //displays dimensions of camera frame
	  //  ROS_INFO("Mat Input Rows: %i", rows );  
       // ROS_INFO("Mat Input Cols: %i", cols );
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


     
   // Update GUI Window
    if (video_received_){ 
        
        cv::imshow(OPENCV_WINDOW, img);
        cv::waitKey(3); //allows for compt to process
        video_received_ = false;
		}
    }

    void detectMovement(){
        //function that publishes to topic how the drone should move when tag seen; executed only when the tag is detected 
		//values are opposite than expected becuase drone must move in opposite vector in order to move the marker correctly
        // e.g.  marker is to the left of center (move_right = TRUE), drone must move left
		  if(move_right){
            
            pos_msg.linear.x = -0.1;
            cout << "-------------   +X   ----------" << endl;

        }else if(move_left){

            pos_msg.linear.x = 0.1;
            cout << "-------------   -X   ----------" << endl;
        }

        if(move_up){

            pos_msg.linear.y = -0.1;
            cout << "-------------   +Y   ----------" << endl;

        }else if(move_down){

            pos_msg.linear.y = 0.1;
            cout << "-------------   -Y   ----------" << endl;

        }


    }


    void send_movement(){
	//publishes proper movement vectors to topic 
        if(land){
            
            pos_msg.angular.x = 1;
            pos_msg.linear.x = 0.0;
            pos_msg.linear.y = 0.0;

        }else{

            pos_msg.angular.x = 0;
        }

        if(tagDetected){
            
            pos_msg.linear.z = 1;
            detectMovement();

        }else{

            pos_msg.linear.z = 0;

        }

       pub_pos.publish(pos_msg);
    }
  
       void run2(){
		//main code runtime procedure
  	   ros::Rate loop_rate(10); // 10 Hz

        while (ros::ok()) { //how code shiuld be structured
  	        //main code
            
          
	        cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F); //identity matrix
   			
   			cv::Mat distanceCoefficients;
   		
   			if(img_received_){  	  
            
		//	 cameraCalibrationProcess(cameraMatrix, distanceCoefficients, img);
            	loadCameraCalibration("Drone2B", cameraMatrix, distanceCoefficients); // filename needs to be changed based on drone used 
                startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension, img);
                send_movement(); //performs function to publish to topic
                
               }
            ros::spinOnce();
            loop_rate.sleep();
    	}
}    
    
  
  
};


int main(int argc, char** argv)
{
    //NOTE: Take into account which file you're using to read camera parameters from!!!
  ros::init(argc, argv, "detect");
  ImageConverter ic;
  ic.run2();
  return 0;
}
