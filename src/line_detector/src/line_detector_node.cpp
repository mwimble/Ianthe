#include "ros/ros.h"
#include "opencv2/videoio.hpp"
#include "line_detector/MazeDetector.h"
#include "line_detector/line_detector.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

bool saveImages = false;
bool showImage = false;

int main( int argc, char** argv ) {
	ros::init(argc, argv, "line_detector_node");
	ros::NodeHandle nh_("~");
	nh_.getParam("save_images", saveImages);
	ROS_INFO("[MazeDetector] PARAM save_images: %s", saveImages ? "TRUE" : "false");
	nh_.getParam("show_image", showImage);
	ROS_INFO("[line_detector_node] PARAM show_image: %s", showImage ? "TRUE" : "false");
	ros::Publisher detectPublisher = nh_.advertise<line_detector::line_detector>("/lineDetect", 1);
	ROS_INFO("[line_detector_node] Starting to spin...");

    VideoCapture cap(0);
    if (!cap.isOpened()) {
    	ROS_ERROR("[line_detector_node] Unable to open VideoCapture device");
     	return -1;
    }
    
    MazeDetector camera = MazeDetector(cap, 0.5);
    if (showImage) camera.createControlWindow();

	const int minAcceptableHorizontalLineLength = 180;
	
	ros::Rate r(20);
	while (ros::ok()) {
		try {
	        camera.updateOriginalImage();
	        camera.thresholdImage();
	        camera.detectLines();
	        
	        Size size = camera.getOriginalImage().size();
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

	        ld.horizontalLeft = camera.horizontalLowerLeftX;
	        ld.horizontalBottom = (camera.horizontalCurve.b * camera.horizontalLowerLeftX) + camera.horizontalCurve.a;
	        ld.horizontalLength = camera.horizontalUpperRightX - camera.horizontalLowerLeftX;
	        ld.horizontalToLeft = (camera.horizontalLowerLeftX < int(size.width * 0.25)) && (ld.horizontalLength > minAcceptableHorizontalLineLength);
	        ld.horizontalToRight = (camera.horizontalUpperRightX > int(size.width * 0.75)) && (ld.horizontalLength > minAcceptableHorizontalLineLength);

	        ld.verticalBottom = camera.verticalLowerLeftY;
	        ld.verticalLeft = (camera.verticalCurve.b * camera.verticalLowerLeftY) + camera.verticalCurve.a;
	        ld.verticalYlength = camera.verticalLowerLeftY - camera.verticalUpperRightY;
	        ld.verticalToBottom = camera.verticalLowerLeftY > int(size.height * 0.75);
	        ld.verticalToTop = camera.verticalUpperRightY < (ld.horizontalBottom - 40);
	        ld.verticalIntercept = ld.verticalCurveA + (ld.verticalCurveB * size.width);

	        ld.header.stamp = currentTime;
	        detectPublisher.publish(ld);

	        if (saveImages || showImage) {
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
	            cv::line(ti, Point(size.width / 2, size.height), Point (size.width / 2, 0), axisColor, 1, 8);
	            cv::line(ti, Point(0, size.height / 2), Point(size.width, size.height / 2), axisColor, 1, 8);
		        cv::line(ti, Point(camera.verticalCurve.b * size.height + camera.verticalCurve.a, size.height), Point(camera.verticalCurve.a, 0), verticalLineColor, 1, 8);
		        cv::line(ti, Point(0, camera.horizontalCurve.a), Point(size.width, camera.horizontalCurve.b * size.width + camera.horizontalCurve.a), verticalLineColor, 1, 8);
	
		        if (showImage) {
			        imshow("Original Image", camera.getOriginalImage());
			        imshow("Thresholded Image", camera.getThresholdedImage());
			        if (waitKey(3) == 27) { return 0; }
			    }

				if (saveImages) {
					ros::Time currentTime = ros::Time::now();
					double secs = currentTime.toSec();
					char fn[128];
					sprintf(fn, "/home/pi/images/%-20.9f.jpg", secs);
					imwrite(fn, camera.getOriginalImage());
				}
		    }

			ros::spinOnce();
			r.sleep();
		} catch(...) {
		    ROS_ERROR("[line_detector_node] Unhandled exception");
        }
	}

	return 0;
}