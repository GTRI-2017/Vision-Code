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

#include <sstream>
#include <iostream>
#include <fstream>


// header files

using namespace cv;
using namespace std;
using std::cout;
using std::endl;

const float calibrationSquareDimension = .0270f; //meters (approx.)
const float arucoSquareDimension = 0.032f; //needs to be standard
const Size chessboardDimensions = Size(9,6);
const int videoPort = 0;



void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners){

    for(int i = 0; i < boardSize.height; i++){

        for(int j=0; j < boardSize.width; j++){
            corners.push_back(Point3f(j  * squareEdgeLength, i * squareEdgeLength, 0.0f)); //z=0 in flat planes
        }
    }
}

//                                              vector of vectors, passed by reference
void getChesssboardCorners(vector<Mat> images, vector<vector<Point2f> >& allFoundCorners, bool showResults = false){

    for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++){

        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //bitwise or operator

        if(found){

            allFoundCorners.push_back(pointBuf);
        }

        if(showResults){ //draws corners onto image

            drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
            imshow("Looking for Corners", *iter);
            waitKey(0);
        }

    }

}



int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, Mat& frame){

    

    vector<int> markerIds;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    Ptr<aruco::Dictionary> markerDictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0)); //0 = DICT_4x4_50




    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    vector<Vec3d> rotationVectors, translationVectors;

    while(true){


        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix,
                          distanceCoefficients, rotationVectors, translationVectors);
        for(int i = 0;  i < markerIds.size(); i ++){

            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
            cout << "---------------------Loop: " << i << "-----------" << endl;
            cout << "Translation Vectors: " << translationVectors[i] << endl;
            cout << "Rotation vectors: " <<  rotationVectors[i] << endl;
		
            //Output x,y,z coordinate values 
	    switch (i){
            case 1: ROS_INFO("X value: %f", translationVectors[i]; break;
            case 2: ROS_INFO("Y value: %f", translationVectors[i]; break;
            case 3: ROS_INFO("Height: %fm", translationVectors[i]; break;
            default: ROS_INFO("Error: i>2, not a possible dimensions"); break;
            } 
		
        }

        imshow("Webcam",  frame);
        if (waitKey(30) >= 0) break; //if nothing is detected within 30 seconds, program stops

    }
    return 1;
}


void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients){

    vector<vector<Point2f> > checkerboardImageSpacePoints;
    getChesssboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    vector<vector<Point3f> > worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
    //  ↑↑↑ resizes and fills vector with  each wSCP points

    vector<Mat> rVectors, tVectors; //radial and tangential vectors
    distanceCoefficients = Mat::zeros(8,1, CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix,
                    distanceCoefficients, rVectors, tVectors );
}




bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients){
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

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients){

    ifstream inStream(name.c_str()); //for C++03
    if (inStream){

        uint64_t rows, columns;

        inStream >> rows; //reads values from the file
        inStream >> columns;

        cameraMatrix = Mat(Size(columns, rows), CV_64F);

        std::cout << "cameraMatrix values" << endl;
        //iterate over cameraMatrix values
        for(int r = 0; r < rows; r++){

            for(int c= 0; c <columns; c++){

                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r,c) = read;

                cout << cameraMatrix.at<double>(r,c) << "\n";
            }
        }
        std::cout << "For loop completed - CAMERA MAT" << endl;

        //grab all the distanceCoefficients

        inStream >> rows;
        inStream >> columns;

        distanceCoefficients = Mat(Size(columns, rows), CV_64F);

        cout << "Distance Coefficients" << endl;
        cout << "Rows: " << rows << endl;
        cout << "Cols: " << columns << endl;

        for(int r = 0; r < rows; r++){

            for(int c= 0; c < columns; c++){

                cout << "Current Col(1) :" << c << endl;
                cout << "Current Row(1) :" << r << endl;
                double read = 0.0f;
                inStream >> read;
                cout << "read done" << endl;
                distanceCoefficients.at<double>(r,c) = read;
                cout << "finding coefficients done " << endl;
                cout << distanceCoefficients.at<double>(r,c) << "\n";

                 }

                cout << "Current Row(2) :" << r << endl;
           }

        std::cout << "For loop completed - DISTANCE COEFFICIENTS" << endl;

        inStream.close();
        return true;
        }

    return false;



}

 vector<Mat> savedImages;

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients, Mat& frame){ //new
   // ROS_INFO("-------------------------cameraCalibrationProcess");
    Mat drawToFrame;

		
   
    vector<vector<Point2f> > markerCorners, rejectedCandidates;

    int framesPerSecond = 20;

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
  //  ROS_INFO("Window Drawn");

  //  while(true){
	
  	//		ROS_INFO("While loop entered - CCP");
        vector<Vec2f> foundPoints;
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
        char character = waitKey(1000 / framesPerSecond); //capture typed chars
       // ROS_INFO("BEFOR SWTICH STATEMENT");
        switch(character){
        
        case ' ':
            //SPACE - saving image
            ROS_INFO("SPACBAR ----------------------------------------SPACEBAR----");
            if(found){
                Mat temp;
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
                saveCameraCalibration("CalibrationNation", cameraMatrix, distanceCoefficients);
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
 
  ros::NodeHandle nh_;
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
        rows = s.height;//240
        cols = s.width;//320
       // ROS_INFO("Mat Input Rows: %i", rows );  
      //  ROS_INFO("Mat Input Cols: %i", cols );
      
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
    
 


    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());


  
 
  void run2(){

  	   ros::Rate loop_rate(10); // 10 Hz

        while (ros::ok()) { //how code shiuld be structured
  	        //main code
            
          //  ROS_INFO("run2 called ");
	        Mat cameraMatrix = Mat::eye(3,3, CV_64F); //identity matrix
   			//ROS_INFO("Mat 1 done");
   			Mat distanceCoefficients;
   		//	ROS_INFO("Mat 2 done");
   		//	ROS_INFO("---------------------------------img receieved ============= %i", img_received_ );
   			if(img_received_){  	  
                 //  ROS_INFO("IF LOOP ENTED IN RUN2");
			   // cameraCalibrationProcess(cameraMatrix, distanceCoefficients, img);
               	loadCameraCalibration("CalibrationNation", cameraMatrix, distanceCoefficients);
                std::cout << "load calibration completed" << endl;
                startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension, img);
                std::cout << "webcam monitoring" << endl;
	           // ROS_INFO("Calibration command done - exectued  ===================================");
                img_received_ = false;
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
  ros::init(argc, argv, "detect");
  ImageConverter ic;
  ic.run2();
  //run();
  
  
  return 0;
}


