#ifndef __MAZEDETECTOR_H
#define __MAZEDETECTOR_H

#include <iostream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class MazeDetector {
public:
	typedef enum { V, H } TLINE_DIRECTION;

	static const char* TLINE_DIRECTION_STRINGS[2];

	struct TLINE_SEGMENT {
	    int x;
	    int y;
	    int length;
	    TLINE_DIRECTION dir;

	    TLINE_SEGMENT(int xx, int yy, int ll, TLINE_DIRECTION dd) :
	        x(xx), y(yy), length(ll), dir(dd) {};

	    std::string toString() const;
	};

	struct CURVE_FIT {
		float a;
		float b;
		CURVE_FIT() : a(0.0), b(0.0) {}
		CURVE_FIT(float a_, float b_) : a(a_), b(b_) {}
	};

	MazeDetector(cv::VideoCapture videoDevice);

	MazeDetector(const std::string testFileName);

	~MazeDetector();

    static const bool debug_LinearCurveFit = false;
	static CURVE_FIT linearCurveFit(std::vector<TLINE_SEGMENT> lineSegments);

	void detectLines();

	cv::Mat getOriginalImage() { return originalImage; }

	cv::Mat getRgbImage() { return rgb_image; }

	double getScaleFactor() { return scaleFactor; }

	bool imageFound() { return imageLoaded; }

	void kmeansImage();

	void updateOriginalImage();

	int horizontalLowerLeftX;
	int horizontalLowerLeftY;
	int horizontalUpperRightX;
	int horizontalUpperRightY;

	int verticalLowerLeftX;
	int verticalLowerLeftY;
	int verticalUpperRightX;
	int verticalUpperRightY;
	CURVE_FIT verticalCurve;
	CURVE_FIT horizontalCurve;
	
private:
	bool debug_;

	ros::NodeHandle nh_;

	// Either use a test file name or a video feed.
	std::string fileName;
	cv::VideoCapture videoFeed;
	bool imageLoaded;
	double scaleFactor;

	// OpenCV objects.
	cv::Mat originalImage;
	cv::Mat grey_image;
	cv::Mat hsvImage;
	cv::Mat reshaped_image32f;
	cv::Mat rgb_image;


	cv::Size	morphSize;

	// Control window variables.
	std::string controlWindowName;

	// For line detection.
	static const int kMinimumLineSegmentLength = 15;

	void computeVerticalLines();
	bool pixelRepresentsALine(uchar pixel) { return pixel == 255; }

	void scaleOriginalImage(double scaleFactor);

};
#endif