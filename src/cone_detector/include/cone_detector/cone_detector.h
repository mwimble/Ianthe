#ifndef __CONE_DETECTOR
#define __CONE_DETECTOR
#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include <string>

//#include "kaimi_mid_camera/kaimi_mid_camera_paramsConfig.h"

using namespace std;
using namespace cv;

class ConeDetector {
private:
	// Detector products.
	int image_height_;
	int image_width_;
	int object_area_;
	bool object_detected_;
	int object_x_;
	int object_y_;

	// ROS node handle.
	ros::NodeHandle nh_;

	// HSV Values and contour area range for the sample thresholding operation.
	int low_hue_range_;
	int high_hue_range_;

	int low_saturation_range_;
	int high_saturation_range_;

	int low_value_range_;
	int high_value_range_;

	int low_contour_area_;
	int high_contour_area_;
	// End values for the sample thresholding operation.

	// Name of camera, to get camera properties.
	string camera_name_;

	// Subscriber to images on "image_topic_name_" topic.
	image_transport::Subscriber image_sub_;

	// Topic publishing the image that might contain a traffic cone.
	string image_topic_name_;

	// OpenCV image transport.
	image_transport::ImageTransport it_;

	// Parameter-settable to show X-windows debugging windows.
	bool show_debug_windows_;

	// Parameter-settable to show times taken for interesting steps in object recognition.
	bool show_step_times_;

	// Publisher handles.
	ros::Publisher cone_found_pub_;

//	dynamic_reconfigure::Server<kaimi_mid_camera::kaimi_mid_camera_paramsConfig> dynamicConfigurationServer;
//	dynamic_reconfigure::Server<kaimi_mid_camera::kaimi_mid_camera_paramsConfig>::CallbackType f;
//
//	static void configurationCallback(kaimi_mid_camera::kaimi_mid_camera_paramsConfig &config, uint32_t level);

	// Process one image.
	void imageCb(Mat& image);

	// Process one image topic message.
	void imageTopicCb(const sensor_msgs::ImageConstPtr& msg);

	// singleton pattern.
	ConeDetector();
	ConeDetector(ConeDetector const&) : it_(nh_) {};
	ConeDetector& operator=(ConeDetector const&) {};

public:
	// Obtain the single instance of this class.
	static ConeDetector& singleton();

	int imageHeight() { return image_height_; }
	int imageWidth() { return image_width_; }
	int objectArea() { return object_area_; }
	bool objectDetected() { return object_detected_; }
	int objectX() { return object_x_; }
	int objectY() { return object_y_; }
};

#endif
