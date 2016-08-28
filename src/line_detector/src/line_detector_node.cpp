#include "ros/ros.h"
#include "opencv2/videoio.hpp"
#include "line_detector/MazeDetector.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

int main( int argc, char** argv ) {
	ros::init(argc, argv, "line_detector_node");
	ros::NodeHandle *rosNode = new ros::NodeHandle();
	ROS_INFO("[line_detector_node] Starting to spin...");

    VideoCapture cap(0);
    if (!cap.isOpened()) {
    	ROS_ERROR("[line_detector_node] Unable to open VideoCapture device");
     	return -1;
    }
    
    MazeDetector camera = MazeDetector(cap, 1.0);
    camera.createControlWindow();

	ros::Rate r(1);
	while (ros::ok()) {
		try {
	        camera.updateOriginalImage();
	        camera.thresholdImage();
	        camera.detectLines();
	        std::vector<MazeDetector::TLINE> verticalLines = camera.getVerticalLines();
	        // cout << "VERTICAL LINES all count: " << verticalLines.size() << endl;
	        Mat ti = camera.getOriginalImage();
	        // int colorArrayLength = sizeof(colorArray) / sizeof(colorArray[0]);
	        // for (int i = 0; i < verticalLines.size(); i++) {
	        //     const MazeDetector::TLINE line = verticalLines[i];
	        //     const MazeDetector::TLINE_SEGMENT& first = line.lineSegments.front();
	        //     const MazeDetector::TLINE_SEGMENT& last = line.lineSegments.back();
	        //     if (line.lineSegments.size() > 5) {
	        //         cout << verticalLines[i].toString() << endl;
	        //         MazeDetector::TLINE::CURVE_FIT curveFit = verticalLines[i].linearCurveFit();
	        //         cout << "...Curve fit a: " << curveFit.a << ", b: " << curveFit.b << endl;
	        //         for (int segx = 0; segx < line.lineSegments.size(); segx++) {
	        //             const MazeDetector::TLINE_SEGMENT seg = line.lineSegments[segx];
	        //             cv::line(ti, Point(seg.x, seg.y), Point(seg.x - 1, seg.y + 1), colorArray[i % colorArrayLength], 1, 8);
	        //             cv::line(ti, Point(seg.x, seg.y), Point(seg.x + seg.length - 1, seg.y + 1), colorArray[i % colorArrayLength], 1, 8);
	        //         }
	        //         //linearCurveFit(line);
	        //         /*
	        //         cv::line(ti, Point(first.x, first.y), Point(last.x, last.y), colorArray[i % colorArrayLength], 2, 8);
	        //         cv::line(ti, Point(last.x, last.y), Point(last.x + last.length, last.y), colorArray[i % colorArrayLength], 2, 8);
	        //         cv::line(ti, Point(last.x + last.length, last.y), Point(first.x + first.length, first.y), colorArray[i % colorArrayLength], 2, 8);
	        //         cv::line(ti, Point(first.x + first.length, first.y), Point(first.x, first.y), colorArray[i % colorArrayLength], 2, 8);
	        //         */
	        //     }
	        // }

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
	        if (verticalLines.size() > 0) {
	            MazeDetector::TLINE::CURVE_FIT curveFit = verticalLines[0].linearCurveFit();
	            cv::line(ti, Point(size.width / 2, size.height), Point (size.width / 2, 0), axisColor, 1, 8);
	            cv::line(ti, Point(0, size.height / 2), Point(size.width, size.height / 2), axisColor, 1, 8);
	            cv::line(ti, Point(curveFit.a + curveFit.b * size.height, size.height), Point(curveFit.a , 0), verticalLineColor, 1, 8);
	        }
	        
	        imshow("Original Image", camera.getOriginalImage());
	        imshow("Thresholded Image", camera.getThresholdedImage());

	        if (waitKey(30) == 27) { return 0; }

			ros::spinOnce();
			r.sleep();
		} catch(...) {
		    ROS_ERROR("[line_detector_node] Unhandled exception");
        }
	}

	return 0;
}