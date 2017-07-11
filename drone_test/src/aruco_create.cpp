//ARUCO Create

#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <ros/ros.h>

#include <sstream>
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;

const float calibrationSquareDimension = .0270f; //meters (approx.)
const float arucoSquareDimension = 0.032f; //needs to be standard
const Size chessboardDimensions = Size(9,6);



void createArucoMarkers(){

    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = //template class
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0));
    for (int i=0; i<50;i++){

        aruco::drawMarker(markerDictionary, i, 500, outputMarker,1);
        ostringstream convert; //used to convert ints to string
        string imageName = "4x4Marker_";
        convert << imageName << i << ".jpg"; //concates the filename
        imwrite(convert.str(), outputMarker);
    }

}



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


int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension){

    Mat frame;

    vector<int> markerIds;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    Ptr<aruco::Dictionary> markerDictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0)); //0 = DICT_4x4_50

    VideoCapture vid(0);

    if(!vid.isOpened()){
        std::cout << "video crashed" << endl;
        return -1;
    }

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    vector<Vec3d> rotationVectors, translationVectors;

    while(true){

        if(!vid.read(frame))
            break;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix,
                          distanceCoefficients, rotationVectors, translationVectors);
        for(int i = 0;  i < markerIds.size(); i ++){

            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
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
    ofstream outStream(name.c_str());
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

              //  cout << "Current Col(1) :" << c << endl;
              //  cout << "Current Row(1) :" << r << endl;
                double read = 0.0f;
                inStream >> read;
              //  cout << "read done" << endl;
                distanceCoefficients.at<double>(r,c) = read;
              //  cout << "finding coefficients done " << endl;
                cout << distanceCoefficients.at<double>(r,c) << "\n";

                 }

             //   cout << "Current Row(2) :" << r << endl;
           }

        std::cout << "For loop completed - DISTANCE COEFFICIENTS" << endl;

        inStream.close();
        return true;
        }

    return false;



}


void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients){

    Mat frame, drawToFrame;


    vector<Mat> savedImages;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if(!vid.isOpened()){ //stops program if video can't be opened
        return; // 0;
    }

    int framesPerSecond = 20;

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    while(true){

        if(!vid.read(frame)) //if frame can't be read
            break;

        vector<Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if(found)
            imshow("Webcam", drawToFrame);
        else
            imshow("Webcam", frame);
        char character = waitKey(1000 / framesPerSecond); //capture typed chars

        switch(character){

        case ' ':
            //SPACE - saving image
            if(found){
                Mat temp;
                frame.copyTo(temp);
                savedImages.push_back(temp);
                std::cout << "SPACEBAR pressed - Image: " << (savedImages.size())<< endl;
            }
            break;
        case 10:
            //ENTER - start calibration
            std::cout << "ENTER Pressed";
            if (savedImages.size() > 15){

                std::cout << "Starting calibration..." << endl;
                cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                saveCameraCalibration("CalibrationNation", cameraMatrix, distanceCoefficients);
                std::cout << "Finishing calibration" << endl;
            }
            break;
        case 27:
            //ESCAPE - exit

            std::cout << "calibration finished" << endl;
            return; // 0;
            break;

        }
    }
}

void run(){



              // while (ros::ok()) {
                              	
								//		ros::Rate loop_rate(10); // 10 Hz
						       	Mat cameraMatrix = Mat::eye(3,3, CV_64F); //identity matrix
										Mat distanceCoefficients;
										cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
								//	  loadCameraCalibration("/home/robotics/opencv_apps/projects/build-aruco_create-Desktop_Qt_5_7_0_GCC_64bit-Debug/CalibrationNation", cameraMatrix, distanceCoefficients);
									 std::cout << "load calibration completed" << endl;
								//		startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);
										std::cout << "webcam monitoring done" << endl;
				 				  
                 /*   ros::spinOnce();
                    loop_rate.sleep();
}*/
}


int main(int argc, char** argv) //need to recalibrate
{
    //createArucoMarkers();
		ros::init(argc, argv, "aruco_create");
    run();
    

    ros::spin();
    return 0;
}















