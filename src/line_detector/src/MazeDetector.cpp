#include "ros/ros.h"
#include "line_detector/MazeDetector.h"

#include <cv.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>   
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include "line_detector/line_detector.h"

using namespace cv;
using namespace std;

static double IMAGE_COLS = 320.0; // Scale image to be this many colums across

void ProccTimePrint(struct timespec start , string msg) {
	struct timespec end;
	clock_gettime(CLOCK_MONOTONIC, &end);
	double duration = ((double)(end.tv_sec - start.tv_sec) * 1.0e9 +
              (double)(end.tv_nsec - start.tv_nsec)) / 1.0e9;

	printf("%s %.4lf(sec) / %.4lf(fps) \n", msg.c_str(),  duration, 1.0 /duration );
	fflush(stdout);
	cout << "- - -" << endl;
} 

void handleControlChange(int newValue, void* userData) {
	// MazeDetector* camera = (MazeDetector*) userData;
	// cout << "New Value" << newValue << ", low hue value: " << camera->getLowHueThreshold() << endl;
}

MazeDetector::MazeDetector(VideoCapture videoDevice)
	: imageLoaded(false), morphSize(5,5), scaleFactor(1.0) {
	nh_ = ros::NodeHandle("~");
	nh_.getParam("debug", debug_);
	ROS_INFO("[MazeDetector] PARAM debug: %s", debug_ ? "TRUE" : "false");
	controlWindowName = "Ianthe Camera Controls";
	setDefaultThresholding();
	videoFeed = videoDevice;
	videoDevice >> originalImage;
	imageLoaded = videoDevice.read(originalImage);
	if (imageLoaded) {
	    // KMEANS takes too long, even on a Jetson, so reduce the image resolution to make it faster.
	    double scaleFactor = IMAGE_COLS / originalImage.cols;
		scaleOriginalImage(scaleFactor);
	    ROS_INFO("[MazeDetector] video capture scaleFactor: %5.3f, original cols: %d, original rows: %d, kmeans cols: %d, kmeans rows: %d",
	    		 scaleFactor, 
	    		 originalImage.cols, 
	    		 originalImage.rows,
	    		 reshaped_image32f.cols,
	    		 reshaped_image32f.rows);
	}
}

MazeDetector::MazeDetector(const std::string testFileName)
	: imageLoaded(false), morphSize(8,8), scaleFactor(1.0) {
	struct stat buffer;

	controlWindowName = "Ewyn Camera Controls";
	setDefaultThresholding();
	fileName = testFileName;
	if (stat (testFileName.c_str(), &buffer) == 0) {
		originalImage = imread(fileName);
		imageLoaded = !originalImage.empty();

	    // KMEANS takes too long, even on a Jetson, so reduce the image resolution to make it faster.
	    double scaleFactor = IMAGE_COLS / originalImage.cols;

		if (imageLoaded) {
			scaleOriginalImage(scaleFactor);
		}
	} else {
		imageLoaded = false;
	}
}

MazeDetector::~MazeDetector() {
	fileName = "";
	videoFeed = VideoCapture();
}

void MazeDetector::createControlWindow() {
	namedWindow(controlWindowName.c_str(), CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar2("Low Hue", controlWindowName.c_str(), &lowHueThreshold, 179, handleControlChange, this);
	cvCreateTrackbar2("High Hue", controlWindowName.c_str(), &highHueThreshold, 179, handleControlChange, this);
	cvCreateTrackbar2("Low Saturation", controlWindowName.c_str(), &lowSaturationThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("High Saturation", controlWindowName.c_str(), &highSaturationThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("Low Value", controlWindowName.c_str(), &lowValueThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("High Value", controlWindowName.c_str(), &highValueThreshold, 255, handleControlChange, this);
}

void MazeDetector::detectLines() {
	if (thresholdedImage.empty()) {
		throw new std::string("No thresholded image to use");
	}

	const int minAcceptableLineSize = 35 * scaleFactor;
	const int maxAcceptableLineSize = 105 * scaleFactor;

	verticalLowerLeftX = 99999;
	verticalLowerLeftY = -1;
	verticalUpperRightX = -1;
	verticalUpperRightY = 99999;
	int startOfContiguousColumnsOfWhitePixels = -1;
	std::vector<TLINE_SEGMENT> verticalLineSegments;
	int midpoint = -1;
	for (int row = thresholdedImage.rows - 1; row >= (thresholdedImage.rows / 2); row--) {
		int lengthOfContiguousColumnsOfWhitePixels = 0;
		for (int col = 0; col < thresholdedImage.cols; col++) {
			if (pixelRepresentsALine(thresholdedImage.at<uchar>(row, col))) {
				if (startOfContiguousColumnsOfWhitePixels == -1) {
					// Capture the start only once.
					startOfContiguousColumnsOfWhitePixels = col;
					lengthOfContiguousColumnsOfWhitePixels = 1;
				} else {
					// This is a continuation of the segment.
					lengthOfContiguousColumnsOfWhitePixels++;
				}
			} else if (startOfContiguousColumnsOfWhitePixels != -1) {
				// End of a contiguous line of pixels.
				if ((lengthOfContiguousColumnsOfWhitePixels >= minAcceptableLineSize) &&
					(lengthOfContiguousColumnsOfWhitePixels <= maxAcceptableLineSize) &&
					((midpoint == -1) ||
					 (abs(midpoint - (startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2))) < 70))) {
					// Seems like a part of a line stripe.
					if (row > verticalLowerLeftY) verticalLowerLeftY = row;
					if (row < verticalUpperRightY) verticalUpperRightY = row;
					if (startOfContiguousColumnsOfWhitePixels < verticalLowerLeftX) verticalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
					if (col > verticalUpperRightX) verticalUpperRightX = col;
					midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
					startOfContiguousColumnsOfWhitePixels = -1;
					TLINE_SEGMENT seg = TLINE_SEGMENT(col, row, lengthOfContiguousColumnsOfWhitePixels, V);
					verticalLineSegments.push_back(seg);
					break;
				} else {
					startOfContiguousColumnsOfWhitePixels = -1;
				}
			}
		} // for (int col...)
	} // for (int row)...

	verticalCurve = linearCurveFit(verticalLineSegments);
	if (debug_) ROS_INFO("[MazeDetector] verticalLowerLeftX: %d, verticalLowerLeftY: %d, verticalUpperRightX: %d, verticalUpperRightY: %d, w: %d, l: %d",
		verticalLowerLeftX, verticalLowerLeftY, verticalUpperRightX, verticalUpperRightY, verticalUpperRightX - verticalLowerLeftX, verticalLowerLeftY - verticalUpperRightY);
	if (debug_) ROS_INFO("[MazeDetector] verticalCurve a: %6.4f, b: %6.4f", verticalCurve.a, verticalCurve.b);

	horizontalLowerLeftX = 99999;
	horizontalLowerLeftY = -1;
	horizontalUpperRightX = -1;
	horizontalUpperRightY = 99999;
	int startOfContiguousRowsOfWhitePixels = -1;
	std::vector<TLINE_SEGMENT> horizontalLineSegments;
	midpoint = -1;
	for (int col = 0; col < thresholdedImage.cols; col++) {
		int lengthOfContiguousRowsOfWhitePixels = 0;
		for (int row = thresholdedImage.rows - 1; row >= (thresholdedImage.rows / 2); row--) {
			if (pixelRepresentsALine(thresholdedImage.at<uchar>(row, col))) {
				if (startOfContiguousRowsOfWhitePixels == -1) {
					// Capture the start only once.
					startOfContiguousRowsOfWhitePixels = row;
					lengthOfContiguousRowsOfWhitePixels = 1;
				} else {
					// This is a continuation of the segment.
					lengthOfContiguousRowsOfWhitePixels++;
				}
			} else if (startOfContiguousRowsOfWhitePixels != -1) {
				// End of a contiguous line of pixels.
				if ((lengthOfContiguousRowsOfWhitePixels >= minAcceptableLineSize) &&
					(lengthOfContiguousRowsOfWhitePixels <= maxAcceptableLineSize) &&
					((midpoint == -1) ||
					 (abs(midpoint - (startOfContiguousRowsOfWhitePixels - (lengthOfContiguousRowsOfWhitePixels / 2))) < 70))) {
					// Seems like a part of a line stripe.
					if (startOfContiguousRowsOfWhitePixels > horizontalLowerLeftY) horizontalLowerLeftY = startOfContiguousRowsOfWhitePixels;
					if (row < horizontalUpperRightY) horizontalUpperRightY = row;
					if (col < horizontalLowerLeftX) horizontalLowerLeftX = col;
					if (col > horizontalUpperRightX) horizontalUpperRightX = col;
					midpoint = startOfContiguousRowsOfWhitePixels - (lengthOfContiguousRowsOfWhitePixels / 2);
					startOfContiguousRowsOfWhitePixels = -1;
					TLINE_SEGMENT seg = TLINE_SEGMENT(col, row, lengthOfContiguousRowsOfWhitePixels, H);
					horizontalLineSegments.push_back(seg);
					break;
				} else {
					startOfContiguousRowsOfWhitePixels = -1;
				}
			}
		} // for (int row...)
	} // for (int col)...

	horizontalCurve = linearCurveFit(horizontalLineSegments);
	if (debug_) ROS_INFO("[MazeDetector] horizontalLowerLeftX: %d, horizontalLowerLeftY: %d, horizontalUpperRightX: %d, horizontalUpperRightY: %d, w: %d, l: %d",
		horizontalLowerLeftX, horizontalLowerLeftY, horizontalUpperRightX, horizontalUpperRightY, horizontalLowerLeftY - horizontalUpperRightY, horizontalUpperRightX - horizontalLowerLeftX);
	if (debug_) ROS_INFO("[MazeDetector] horizontalCurve a: %6.4f, b: %6.4f", horizontalCurve.a, horizontalCurve.b);
}

void MazeDetector::kmeansImage() {
	struct timespec start;
    Mat labels;
    int cluster_number = 4; // 10; // This is arbitrary. I increased it until I got the results I wanted.
    TermCriteria criteria {TermCriteria::COUNT, 100, 1};

	clock_gettime(CLOCK_MONOTONIC, &start);
    Mat centers;
    kmeans(reshaped_image32f, cluster_number, labels, criteria, 1, KMEANS_RANDOM_CENTERS, centers);
	ProccTimePrint(start , "kmeans");
}

void MazeDetector::scaleOriginalImage(double scaleFactor) {
	if (!originalImage.empty() && (scaleFactor != 1.0)) {
		// Size scaleSize = originalImage.size();
		// scaleSize.height *= scaleFactor;
		// scaleSize.width *= scaleFactor;
		// resize(originalImage, originalImage, scaleSize);

	    Mat resizedImage;
	    resize(originalImage, resizedImage, Size(), scaleFactor, scaleFactor, INTER_AREA);

	    Mat reshaped_image = resizedImage.reshape(1, resizedImage.cols * resizedImage.rows);
	    cout << "reshaped image: " << reshaped_image.rows << ", " << reshaped_image.cols << endl;
	    assert(reshaped_image.type() == CV_8UC1);

	    reshaped_image.convertTo(reshaped_image32f, CV_32FC1, 1.0 / 255.0);
	    cout << "reshaped image 32f: " << reshaped_image32f.rows << ", " << reshaped_image32f.cols << endl;
	    assert(reshaped_image32f.type() == CV_32FC1);
	}
}

void MazeDetector::setDefaultThresholding() {
	lowHueThreshold = kLowHueThreshold;
	highHueThreshold = kHighHueThreshold;
	lowSaturationThreshold = kLowSaturationThreshold;
	highSaturationThreshold = kHighSaturationThreshold;
	lowValueThreshold = kLowValueThreshold;
	highValueThreshold = kHighValueThreshold;
}

void MazeDetector::thresholdImage() {
	Mat hsvImage;
	cvtColor(originalImage, hsvImage, COLOR_BGR2HSV);
	inRange(hsvImage, 
			Scalar(lowHueThreshold, lowSaturationThreshold, lowValueThreshold),
			Scalar(highHueThreshold, highSaturationThreshold, highValueThreshold),
			thresholdedImage);

	// Remove small objects from the forground.
	erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
	dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));

	// Fill small holes in the background;
	// dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
	// erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
}

void MazeDetector::updateOriginalImage() {
	if (fileName.size() > 0) {
		originalImage = imread(fileName);
		imageLoaded = !originalImage.empty();
		if (imageLoaded) {
			scaleOriginalImage(scaleFactor);
		}
	} else {
		imageLoaded = videoFeed.read(originalImage);
		if (imageLoaded) {
			scaleOriginalImage(scaleFactor);
		}
	}
}

MazeDetector::CURVE_FIT MazeDetector::linearCurveFit(std::vector<TLINE_SEGMENT> lineSegments) {
    float sumx = 0;
    float sumx2 = 0;
    float sumy = 0;
    float sumxy = 0;
    int n = lineSegments.size();
    for (int i = 0; i < n; i++) {
    	TLINE_SEGMENT lineSegment = lineSegments[i];
    	int x;
    	int y;
    	if (lineSegment.dir == H) {
    		y = lineSegment.y + (lineSegment.length / 2);
    		x = lineSegment.x;
    	} else {
    		y = lineSegment.x - (lineSegment.length / 2);
    		x = lineSegment.y;
    	}

        sumx = sumx + x;
        sumx2 = sumx2 + x * x;
        sumy = sumy + y;
        sumxy = sumxy + x * y;
//        if (debug_LinearCurveFit) { cout << x << "\t" << y << endl; }
    }

    float t1 = sumx2 * sumy;
    float t2 = sumx * sumxy;
    float num = t1 - t2;
    float t3 = n * sumx2;
    float t4 = sumx * sumx;
    float denom = t3 - t4;
    float a = num / denom;
    float b = ((n * sumxy) - (sumx * sumy)) / ((n * sumx2) - (sumx * sumx));
    if (debug_LinearCurveFit) {
        cout << "n: " << n 
        	 << ", sumx: " << sumx
        	 << ", sumx2: " << sumx2
        	 << ", sumy: " << sumy
        	 << ", sumxy: " << sumxy
        	 << ", t1: " << t1 
        	 << ", t2: " << t2
        	 << ", num: " << num 
        	 << ", t3: " << t3 
        	 << ", t4: " << t4 
        	 << ", denom: " << denom 
        	 << ", a: " << a 
        	 << ", b: " << b << endl;
    }

    return CURVE_FIT(a, b);
}

std::string MazeDetector::TLINE_SEGMENT::toString() const {
    std::stringstream ss;
    ss << "{"
         << "x:" << x
         << ", y:" << y
         << ", l:"  << length
         << ", d: " << dir
         << "}";
    return ss.str();
}