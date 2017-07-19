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
/*Camera Dimensions (x & y):
    Drone1B: 144 x 174

*/
 


#include <sstream>
#include <iostream>
#include <fstream>
//NOTE:: In source file, if function is not static, it's an instance function

// header files


using namespace std;
using std::cout;
using std::endl;

const float calibrationSquareDimension = .0270f; //meters (approx.)
const float arucoSquareDimension = .132; //needs to be standard 0.032f
const cv::Size chessboardDimensions = cv::Size(9,6);
const int videoPort = 0;

//center camera frame, changers between drones:
// P1B: 144 x 174  P2B: 640 x 360
const float x_P1B = 72;
const float y_P1B = 87;
const float x_P2B = 320;
const float y_P2B = 180;



const float x_center = x_P2B;
const float y_center = y_P2B;
const float error = 25;
const float x_ideal_minimum = x_center - error;
const float x_ideal_maximum = x_center + error;
const float y_ideal_lower = y_center + error;
const float y_ideal_higher = y_center - error;

//movements are in reference to the bottom camera 
bool move_right = false;
bool move_left = false;
bool move_up = false;
bool move_down = false;
bool tagDetected = false; 
bool land = false;




void createKnownBoardPosition(cv::Size boardSize, float squareEdgeLength, vector<cv::Point3f>& corners){

    for(int i = 0; i < boardSize.height; i++){

        for(int j=0; j < boardSize.width; j++){
            corners.push_back(cv::Point3f(j  * squareEdgeLength, i * squareEdgeLength, 0.0f)); //z=0 in flat planes
        }
    }
}

//                                              vector of vectors, passed by reference
void getChesssboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f> >& allFoundCorners, bool showResults = false){

    for(vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++){

        vector<cv::Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //bitwise or operator

        if(found){

            allFoundCorners.push_back(pointBuf);
        }

        if(showResults){ //draws corners onto image

            drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
            imshow("Looking for Corners", *iter);
            cv::waitKey(0);
        }

    }

}

void compareToCenter(float x, float y ){

    if( (x >= x_ideal_minimum && x <= x_ideal_maximum) && (y <= y_ideal_lower && y >= y_ideal_higher)) {
        land = true;
        cout << " -------------------------In Center -----------------------" << endl;
    }else{
        land = false;
        }
    //filters out error readings
   if(x > 0 && y > 0 ){

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
            for(int p=0; p<markerCorners[i].size(); p++) {// You may not have had 10 in here before, only go to size().
            cout << markerCorners[i][p] << " " ;
            x_sum  += markerCorners[i][p].x;
            y_sum  += markerCorners[i][p].y;
            }
            x_average = x_sum/4;
            y_average = y_sum/4;

            cout << "\n Mean X: " << x_average << " Mean Y: " << y_average << endl;
            compareToCenter(x_average, y_average);
        
     /*   cv::Mat mean_;
        cv::reduce(markerCorners, mean_, 01, CV_REDUCE_AVG);
        cv::Point2f mean(mean_.at<float>(0,0), mean_.at<float>(0,1));
        cout << "Mean Value" << mean << endl;         */   

  
        for(int i = 0;  i < markerIds.size(); i ++){

            cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
          //  ROS_INFO("---------------------Loop: %i", i);
           // std::cout << "Translation Vectors: " << translationVectors[i] << std::endl;
            //   std::cout << "Translation Vectors: %i" << translationVectors[i] << std::endl;
         //  std::cout << "Rotation vectors: " <<  rotationVectors[i] << std::endl;
        // std::cout << "Corners: " <<  markerCorners[i]  [i] << std::endl;
        }
        //print vectors contents
        /*for( vector<vector<vector<cv::Point2f> > >::const_iterator i = markerCorners.begin(); i != markerCorners.end(); ++i)
{
   for( vector<vector<cv::Point2f> >::const_iterator j = i->begin(); j != i->end(); ++j)
   {
        for( <vector<cv::Point2f>::const_iterator k = j->begin(); k != j->end(); ++k)
        { 
                 cout<<*k<<' ';
        }
   }
} */

      

        imshow("Webcam",  frame);
        if (cv::waitKey(30) >= 0) break; //if nothing is detected within 30 seconds, program stops

    }
    return 1;
}



void cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients){

    vector<vector<cv::Point2f> > checkerboardImageSpacePoints;
    getChesssboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    vector<vector<cv::Point3f> > worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
    //  ↑↑↑ resizes and fills vector with  each wSCP points

    vector<cv::Mat> rVectors, tVectors; //radial and tangential vectors
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

        outStream << rows << endl; //saves data into the file
        outStream << columns << endl;

        for(int r =0; r < rows; r++){

            for(int c = 0; c < columns; c++){

                double value = cameraMatrix.at<double>(r,c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        outStream << rows << endl; //saves data into the file
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

    ifstream inStream(name.c_str()); //for C++03
    if (inStream){

        uint64_t rows, columns;

        inStream >> rows; //reads values from the file
        inStream >> columns;

        cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

      //  std::cout << "cameraMatrix values" << endl;
        //iterate over cameraMatrix values
        for(int r = 0; r < rows; r++){

            for(int c= 0; c <columns; c++){

                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r,c) = read;

               // cout << cameraMatrix.at<double>(r,c) << "\n";
            }
        }
        //std::cout << "For loop completed - CAMERA MAT" << endl;

        //grab all the distanceCoefficients

        inStream >> rows;
        inStream >> columns;

        distanceCoefficients = cv::Mat(cv::Size(columns, rows), CV_64F);

       // cout << "Distance Coefficients" << endl;
      //  cout << "Rows: " << rows << endl;
       // cout << "Cols: " << columns << endl;

        for(int r = 0; r < rows; r++){

            for(int c= 0; c < columns; c++){

              //  cout << "Current Col(1) :" << c << endl;
              //  cout << "Current Row(1) :" << r << endl;
                double read = 0.0f;
                inStream >> read;
            //    cout << "read done" << endl;
                distanceCoefficients.at<double>(r,c) = read;
           //     cout << "finding coefficients done " << endl;
            //    cout << distanceCoefficients.at<double>(r,c) << "\n";

                 }

           }

      //  std::cout << "For loop completed - DISTANCE COEFFICIENTS" << endl;

        inStream.close();
        return true;
        }

    return false;
}

 vector<cv::Mat> savedImages;

void cameraCalibrationProcess(cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients, cv::Mat& frame){ //new
   // ROS_INFO("-------------------------cameraCalibrationProcess");
    cv::Mat drawToFrame;

		
   
    vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;

    int framesPerSecond = 20;

    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
  //  ROS_INFO("Window Drawn");

  //  while(true){
	
  	//		ROS_INFO("While loop entered - CCP");
        vector<cv::Vec2f> foundPoints;
        bool found = false; 
			//	ROS_INFO("start to find chessboard");
        if (frame.empty()){
        ROS_INFO("MATRIX IS EMPTY!");
        }else{
            found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                // ROS_INFO("found = %i", found);
        }

        //ROS_INFO("chessboard found");
        frame.copyTo(drawToFrame);
      //  ROS_INFO("Frame Copied");
        
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
     //   ROS_INFO("DRAW called");
        if(found){
            //ROS_INFO("IF statement");
            imshow("Webcam", drawToFrame);
    }else{
           // ROS_INFO("Else statment");
           // ROS_INFO("FRAME IS EMPTY %i", frame.empty());
            imshow("Webcam", frame);  //ERROR bc frame is empty
    }
        char character = cv::waitKey(1000 / framesPerSecond); //capture typed chars
       // ROS_INFO("BEFOR SWTICH STATEMENT");
        switch(character){
        
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
    //}
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
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
       
       // cout << "image sent" << endl;
        img_received_ = true;
        video_received_ = true;
       // cout << "image is true" << endl;
        int rows = img.rows;
        int cols = img.cols;
       // ROS_INFO("A");
        cv::Size s = img.size();
        rows = s.height;//240 for P1 and 360 P2
        cols = s.width;//320 for P1 and 640 for P2
      //  ROS_INFO("Mat Input Rows: %i", rows );  
       // ROS_INFO("Mat Input Cols: %i", cols );
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (img.rows > 60 && img.cols > 60)
     
   // Update GUI Window
   
    if (video_received_){ //Error: not being executed repeatedly
        //cv::circle(img, cv::Point(100, 100), 100, CV_RGB(255,0,0));
        cv::imshow(OPENCV_WINDOW, img);
       // ROS_INFO("imshow called ---------------------------------------- imshow CALLED");
        cv::waitKey(3); //allows for compt to process
        video_received_ = false;
    }
    
    }

    void detectMovement(){
        //function that determines how the drone moves; executed when the tag is detected 
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
  
     // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  void run2(){

  	   ros::Rate loop_rate(10); // 10 Hz

        while (ros::ok()) { //how code shiuld be structured
  	        //main code
            
          //  ROS_INFO("run2 called ");
	        cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F); //identity matrix
   			//ROS_INFO("Mat 1 done");
   			cv::Mat distanceCoefficients;
   		//	ROS_INFO("Mat 2 done");
   		//	ROS_INFO("---------------------------------img receieved ============= %i", img_received_ );
   			if(img_received_){  	  
                 //  ROS_INFO("IF LOOP ENTED IN RUN2");
		//	 cameraCalibrationProcess(cameraMatrix, distanceCoefficients, img);
            	loadCameraCalibration("Drone2B", cameraMatrix, distanceCoefficients);
             //  std::cout << "load calibration completed" << endl;
                startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension, img);
              //  std::cout << "webcam monitoring" << endl;
	           // ROS_INFO("Calibration command done - exectued  ===================================");
                
                send_movement();
                
               }
            ros::spinOnce();
            loop_rate.sleep();
    	}
}    
  /*  
   */
  
};



  

//bool ImageConverter::img_received_ = false;


int main(int argc, char** argv)
{
    //NOTE: Take into account which file you're using to read camera parameters from!!!
  ros::init(argc, argv, "detect");
  ImageConverter ic;
  ic.run2();
  //run();
  
  
  return 0;
}


