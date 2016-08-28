#include "ros/ros.h"
#include "line_detector/MazeDetector.h"

#include <cv.h>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>

using namespace cv;
using namespace std;

bool DEBUG = false;

void handleControlChange(int newValue, void* userData) {
	MazeDetector* camera = (MazeDetector*) userData;
	cout << "New Value" << newValue << ", low hue value: " << camera->getLowHueThreshold() << endl;
}

MazeDetector::MazeDetector(VideoCapture videoDevice, double scaleFactor)
	: imageLoaded(false), morphSize(5,5), scaleFactor(scaleFactor) {
	controlWindowName = "Ewyn Camera Controls";
	setDefaultThresholding();
	videoFeed = videoDevice;
	videoDevice >> originalImage;
	imageLoaded = videoDevice.read(originalImage);
	if (imageLoaded) {
		scaleOriginalImage(scaleFactor);
	}
}

MazeDetector::MazeDetector(const std::string testFileName, double scaleFactor)
	: imageLoaded(false), morphSize(8,8), scaleFactor(scaleFactor) {
	struct stat buffer;

	controlWindowName = "Ewyn Camera Controls";
	setDefaultThresholding();
	fileName = testFileName;
	if (stat (testFileName.c_str(), &buffer) == 0) {
		originalImage = imread(fileName);
		imageLoaded = !originalImage.empty();
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

	// Lines are detected as boxes. The box is built up as a collection of line segments,
	// each of which is a contiguous collection of white pixels in a thresholded image.
	// When we find a line segment of white pixels, we look to see if this seems to be
	// a continuation of a previous line we discovered. If so, we add this line segment
	// to the collection. Else we create a new line and add this segment to it. E.g.,
	// when looking for vertical lines, if the image contains:
	//     xxxx
	//    yyy     zzzz
	// Each of x, y, z represents a line segment of contiguous white pixels for that row
	// in the image. We start the scan at the bottom. When we see yyy, there are no existing lines so
	// we create one and add yyy (i.e., the column number where it starts and its width).
	// Continuing the scanning from the bottom and left to right, we next encounter zzzz. It
	// doesn't appear to be part of the line starting with yyy, so we create a new line and add
	// zzzz to it. Then we go to the next row and find xxx. When we compare the start column
	// and length of xxx to the lines that contain yyy and zzzz, we find that xxxx seems to be
	// a continuation of the line containing yyy but not zzzz. We add xxxx to the line that
	// already contains yyy. We then have two lines, so far. One contains yyy and xxxx. The second
	// only contains zzzz. We continue scanning rows from the bottom to the top, and try to
	// build boxes of related line segments of contiguous white pixels.

	// For detecting horizontal lines, we do the same, keeping a separate collection of boxes
	// for horizontal lines, only we logically flip the image by rotating it left 90 degrees.

	verticalLines = vector<TLINE>();
	horizontalLines = vector<TLINE>();

	// Detect vertical lines.
	// Scan from the last row towards the top, stopping at the halfway point
	// (assumes pixels in the top half of the image are too far away to be of interest).
	int startOfContiguousColumnsOfWhitePixels = -1;
	for (int row = thresholdedImage.rows - 1; row >= (thresholdedImage.rows / 2); row--) {
		int lengthOfContiguousColumnsOfWhitePixels = 0;
		for (int col = 0; col < thresholdedImage.cols; col++) {
			// if (DEBUG) ROS_INFO("[detectLines] ros: %d, col: %d, pixelRepresentsALine: %d, startOfContiguousColumnsOfWhitePixels: %d, lengthOfContiguousColumnsOfWhitePixels: %d", 
			// 			   row, 
			// 			   col,
			// 			   pixelRepresentsALine(thresholdedImage.at<uchar>(row, col)),
			// 			   startOfContiguousColumnsOfWhitePixels, 
			// 			   lengthOfContiguousColumnsOfWhitePixels);
			if (pixelRepresentsALine(thresholdedImage.at<uchar>(row, col))) {
				if (startOfContiguousColumnsOfWhitePixels == -1) {
					// Capture the start only once.
					startOfContiguousColumnsOfWhitePixels = col;
					lengthOfContiguousColumnsOfWhitePixels = 1;
				} else {
					// This is a continuation of the segment.
					lengthOfContiguousColumnsOfWhitePixels++;
				}
			} else if ((startOfContiguousColumnsOfWhitePixels != -1) &&
					   (col < (thresholdedImage.cols - 2)) &&
					   (pixelRepresentsALine(thresholdedImage.at<uchar>(row, col + 1)) ||
					   	pixelRepresentsALine(thresholdedImage.at<uchar>(row, col + 2)))) {
				// Ignore one or two pixel black spots across the line.
				lengthOfContiguousColumnsOfWhitePixels++;
			} else {
				// This is not part of a contiguous range of white pixels.
				// If this column is the first past a detected contiguous range of white pixels,
				// and the range was "interesting" (large enough), capture it as a potential
				// line segment in the collection.
				if ((startOfContiguousColumnsOfWhitePixels >= 0 /* We found a segment*/) &&
					(lengthOfContiguousColumnsOfWhitePixels >= kMinimumLineSegmentLength)) {
					insertLineSegment(verticalLines, 
									  TLINE_SEGMENT(startOfContiguousColumnsOfWhitePixels, 
									  				row, 
									  				lengthOfContiguousColumnsOfWhitePixels,
									  				V));
					startOfContiguousColumnsOfWhitePixels = -1;
					lengthOfContiguousColumnsOfWhitePixels = 0;
				}
			}
		}
	}

	const int minAcceptableLineSize = 35;
	const int maxAcceptableLineSize = 105;
	verticalLowerLeftX = 99999;
	verticalLowerLeftY = -1;
	verticalUpperRightX = -1;
	verticalUpperRightY = 99999;
	/*int*/ startOfContiguousColumnsOfWhitePixels = -1;
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
					if (col < 100) {
						cout << "OOPS" << endl;
					}
					if (row > verticalLowerLeftY) verticalLowerLeftY = row;
					if (row < verticalUpperRightY) verticalUpperRightY = row;
					if (startOfContiguousColumnsOfWhitePixels < verticalLowerLeftX) verticalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
					if (col > verticalUpperRightX) verticalUpperRightX = col;
					midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
					startOfContiguousColumnsOfWhitePixels = -1;
					break;
				} else {
					startOfContiguousColumnsOfWhitePixels = -1;
				}
			}
		} // for (int col...)
	} // for (int row)...

	ROS_INFO("[detectLines] verticalLowerLeftX: %d, verticalLowerLeftY: %d, verticalUpperRightX: %d, verticalUpperRightY: %d, w: %d, l: %d",
		verticalLowerLeftX, verticalLowerLeftY, verticalUpperRightX, verticalUpperRightY, verticalUpperRightX - verticalLowerLeftX, verticalLowerLeftY - verticalUpperRightY);

	horizontalLowerLeftX = 99999;
	horizontalLowerLeftY = -1;
	horizontalUpperRightX = -1;
	horizontalUpperRightY = 99999;
	int startOfContiguousRowsOfWhitePixels = -1;
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
					if (row > 400) {
						cout << "oops" << endl;
					}
					if (startOfContiguousRowsOfWhitePixels > horizontalLowerLeftY) horizontalLowerLeftY = startOfContiguousRowsOfWhitePixels;
					if (row < horizontalUpperRightY) horizontalUpperRightY = row;
					if (col < horizontalLowerLeftX) horizontalLowerLeftX = col;
					if (col > horizontalUpperRightX) horizontalUpperRightX = col;
					midpoint = startOfContiguousRowsOfWhitePixels - (lengthOfContiguousRowsOfWhitePixels / 2);
					startOfContiguousRowsOfWhitePixels = -1;
					break;
				} else {
					startOfContiguousRowsOfWhitePixels = -1;
				}
			}
		} // for (int row...)
	} // for (int col)...

	ROS_INFO("[detectLines] horizontalLowerLeftX: %d, horizontalLowerLeftY: %d, horizontalUpperRightX: %d, horizontalUpperRightY: %d, w: %d, l: %d",
		horizontalLowerLeftX, horizontalLowerLeftY, horizontalUpperRightX, horizontalUpperRightY, horizontalLowerLeftY - horizontalUpperRightY, horizontalUpperRightX - horizontalLowerLeftX);
}

void MazeDetector::insertLineSegment(vector<TLINE>& arrayOfLines, const TLINE_SEGMENT newLine) {
    bool lineFound = false;

    // Iterate over all existing lines as this new line might be part of several
    // existing lines if the lines in the image are not perfectly N, S, E, W.
    for (int i = 0; i < arrayOfLines.size(); i++) {
        if (linesOverlap(arrayOfLines[i].lineSegments.back(), newLine) &&
            (newLine.x >= arrayOfLines[i].midpoint() - arrayOfLines[i].averageLength()) &&
            (newLine.x <= arrayOfLines[i].midpoint() + arrayOfLines[i].averageLength())) {
            arrayOfLines[i].addLineSegment(newLine);
            lineFound = true;
		if (DEBUG) ROS_INFO("[insertLineSegment] lineFound: %d, i: %d, NEWLINE x, %d, y: %d, length: %d, overlap: %d, line midpoint: %d, averageLength: %d"
							, lineFound
							, i
							, newLine.x
							, newLine.y
							, newLine.length
							, linesOverlap(arrayOfLines[i].lineSegments.back(), newLine)
							, arrayOfLines[i].midpoint()
							, arrayOfLines[i].averageLength()
							);
        }
    }

    if (!lineFound) {
        //cout << "Creating new line" << endl;
        TLINE oneElementVector;
        oneElementVector.addLineSegment(newLine);
        oneElementVector.dir = newLine.dir;
        arrayOfLines.push_back(oneElementVector);
    }
}

bool MazeDetector::linesOverlap(const TLINE_SEGMENT l1, TLINE_SEGMENT l2) {
    bool result = false;
    if (l1.x <= l2.x) {
        result = l2.x <= (l1.x + l1.length);
    } else {
        result = l1.x <= (l2.x + l2.length);
    }

    return result;
}

void MazeDetector::scaleOriginalImage(double scaleFactor) {
	if (!originalImage.empty() && (scaleFactor != 1.0)) {
		Size scaleSize = originalImage.size();
		scaleSize.height *= scaleFactor;
		scaleSize.width *= scaleFactor;
		resize(originalImage, originalImage, scaleSize);
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
	dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
	erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
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

const char* MazeDetector::TLINE_DIRECTION_STRINGS[2] = { "V", "H" };
int MazeDetector::TLINE::nextNumber = 0;

MazeDetector::TLINE::TLINE() {
    for (int i = 0; i < kCapturedMidpoints; i++) { lastMidpoints[i] = -1; }
    midpointIndex = 0;
    sumLengths = 0;
    number = nextNumber++;
}

void MazeDetector::TLINE::addLineSegment(TLINE_SEGMENT lineSegment) {
    lineSegments.push_back(lineSegment);
    lastMidpoints[midpointIndex++] = lineSegment.x + (lineSegment.length / 2);
    if (midpointIndex >= 10) midpointIndex = 0;
    sumLengths += lineSegment.length;
    if (debugAddLineElement) {
        std::cout << "Push to existing line[" << number
             << "] back: " << lineSegments.back().toString()
             << ", newLine: " << lineSegment.toString()
             << ", avg midpoint: " << midpoint()
             << ", line midpoint: " << (lineSegment.x + lineSegment.length / 2)
             << ", segment count: " << lineSegments.size()
             << std::endl;
    }
}

MazeDetector::TLINE::CURVE_FIT MazeDetector::TLINE::linearCurveFit() {
    if (debugLinearCurveFit) {
        cout << "linearCurveFit START line[" << number << "]" << endl;
    }

    float sumx = 0;
    float sumx2 = 0;
    float sumy = 0;
    float sumxy = 0;
    int n = lineSegments.size();
    for (int i = 0; i < n; i++) {
    	TLINE_SEGMENT lineSegment = lineSegments[i];
    	int x;
    	int y;
    	if (isVerticalLine()) {
    		y = lineSegment.x + (lineSegment.length / 2);
    		x = lineSegment.y;
    	} else {
    		x = lineSegment.x;
    		y = lineSegment.y;
    	}

        sumx = sumx + x;
        sumx2 = sumx2 + x * x;
        sumy = sumy + y;
        sumxy = sumxy + x * y;
    }

    float t1 = sumx2 * sumy;
    float t2 = sumx * sumxy;
    float num = t1 - t2;
    float t3 = n * sumx2;
    float t4 = sumx * sumx;
    float denom = t3 - t4;
    float a = num / denom;
    float b = ((n * sumxy) - (sumx * sumy)) / ((n * sumx2) - (sumx * sumx));
    if (debugLinearCurveFit) {
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

int MazeDetector::TLINE::midpoint() const {
    int result = 0;
    int count = 0;
    for (int i = 0; i < 10; i++) {
        if (lastMidpoints[i] != -1) {
            result += lastMidpoints[i];
            count++;
        }
    }

    return count == 0 ? 0 : result / count;
} 

std::string MazeDetector::TLINE::toString() {
    std::stringstream ss;
    const TLINE_SEGMENT& first = lineSegments.front();
    const TLINE_SEGMENT& last = lineSegments.back();
    ss << "{Line " << number 
       << ", isVertical: " << (isVerticalLine() ? "T" : "f")
       << ", dir: " << TLINE_DIRECTION_STRINGS[dir]
       << ", segCount: " << lineSegments.size()
       << ", front: " << first.toString()
       << ", back: " << last.toString()
       << ", avg midpoint: " << midpoint()
       << ", avg length: " << averageLength()
       << ", box ll: " << first.x << "," << first.y
       << ", ul: " << last.x << "," << last.y
       << ", lr: " << (first.x + first.length - 1) << "," << first.y
       << ", ur: " << (last.x + last.length - 1) << "," << last.y
       << "}";
    return ss.str();
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