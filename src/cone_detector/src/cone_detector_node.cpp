#include <ros/ros.h>
#include <ros/console.h>

#include "cone_detector/cone_detector.h"
#include "cone_detector/ObjectDetector.h"

int main(int argc, char** argv) {
    ConeDetector* coneDetector;

    ros::init(argc, argv, "cone_detector_node");

    int fps;
	ros::NodeHandle nh;
    cone_detector::ObjectDetector detector_msg;

    ros::param::get("~fps", fps);
    ROS_INFO("[cone_detector] PARAM fps: %d", fps);
    coneDetector = &ConeDetector::singleton();

	ros::Publisher detector_pub = nh.advertise<cone_detector::ObjectDetector>("cone_detector", 2);
    ros::Rate rate(fps);

    while (ros::ok()) {
    	detector_msg.header.stamp = ros::Time::now();
		detector_msg.image_height = coneDetector->imageHeight();
		detector_msg.image_width = coneDetector->imageWidth();
		detector_msg.object_area = coneDetector->objectArea();
		detector_msg.object_detected = coneDetector->objectDetected();
		detector_msg.object_x = coneDetector->objectX();
		detector_msg.object_y = coneDetector->objectY();
		detector_pub.publish(detector_msg);
        rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
