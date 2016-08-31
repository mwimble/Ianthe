#include "ros/ros.h"
#include "opencv2/videoio.hpp"
#include "line_detector/MazeDetector.h"
#include "line_detector/line_detector.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

bool showImage = false;

int main( int argc, char** argv ) {
	ros::init(argc, argv, "line_detector_node");
	ros::param::param<bool>("show_image", showImage, "false");
	ROS_INFO("[line_detector_node] PARAM show_image: %s", showImage ? "TRUE" : "true");
showImage = false; //#####
	ros::NodeHandle nh_;
	ros::Publisher detectPublisher = nh_.advertise<line_detector::line_detector>("/lineDetect", 1);
	ROS_INFO("[line_detector_node] Starting to spin...");

    VideoCapture cap(0);
    if (!cap.isOpened()) {
    	ROS_ERROR("[line_detector_node] Unable to open VideoCapture device");
     	return -1;
    }
    
    MazeDetector camera = MazeDetector(cap, 0.5);
    if (showImage) camera.createControlWindow();

	ros::Rate r(20);
	while (ros::ok()) {
		try {
	        camera.updateOriginalImage();
	        camera.thresholdImage();
	        camera.detectLines();
	        
	        ros::Time currentTime = ros::Time::now();
	        line_detector::line_detector ld;
	        ld.horizontalLowerLeftX = camera.horizontalLowerLeftX;
	        ld.horizontalLowerLeftY = camera.horizontalLowerLeftY;
	        ld.horizontalUpperRightX = camera.horizontalUpperRightX;
	        ld.horizontalUpperRightY = camera.horizontalUpperRightY;
	        ld.horizontalCurveA = camera.horizontalCurve.a;
	        ld.horizontalCurveB = camera.horizontalCurve.b;

	        ld.verticalLowerLeftX = camera.verticalLowerLeftX;
	        ld.verticalLowerLeftY = camera.verticalLowerLeftY;
	        ld.verticalUpperRightX = camera.verticalUpperRightX;
	        ld.verticalUpperRightY = camera.verticalUpperRightY;
	        ld.verticalCurveA = camera.verticalCurve.a;
	        ld.verticalCurveB = camera.verticalCurve.b;

	        ld.header.stamp = currentTime;
	        detectPublisher.publish(ld);

	        if (showImage) {
		        Mat ti = camera.getOriginalImage();

		        if (camera.verticalLowerLeftY != -1) {
		        	cv::rectangle(ti, 
		        			 Point(camera.verticalLowerLeftX, camera.verticalLowerLeftY),
		        			 Point(camera.verticalUpperRightX, camera.verticalUpperRightY), colorArray[0], 3, 8);
		        }

		        if (camera.horizontalLowerLeftY != -1) {
		        	cv::rectangle(ti, 
		        			 Point(camera.horizontalLowerLeftX, camera.horizontalLowerLeftY),
		        			 Point(camera.horizontalUpperRightX, camera.horizontalUpperRightY), colorArray[1], 3, 8);
		        }

		        Scalar axisColor = Scalar(0, 255, 255);
		        Scalar verticalLineColor = Scalar(0, 136, 255);
		        Size size = camera.getOriginalImage().size();
	            cv::line(ti, Point(size.width / 2, size.height), Point (size.width / 2, 0), axisColor, 1, 8);
	            cv::line(ti, Point(0, size.height / 2), Point(size.width, size.height / 2), axisColor, 1, 8);
		        cv::line(ti, Point(camera.verticalCurve.b * size.height + camera.verticalCurve.a, size.height), Point(camera.verticalCurve.a, 0), verticalLineColor, 1, 8);
		        cv::line(ti, Point(0, camera.horizontalCurve.a), Point(size.width, camera.horizontalCurve.b * size.width + camera.horizontalCurve.a), verticalLineColor, 1, 8);
		        
		        imshow("Original Image", camera.getOriginalImage());
		        imshow("Thresholded Image", camera.getThresholdedImage());

		        if (waitKey(30) == 27) { return 0; }
		    }

			ros::spinOnce();
			r.sleep();
		} catch(...) {
		    ROS_ERROR("[line_detector_node] Unhandled exception");
        }
	}

	return 0;
}