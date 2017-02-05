#include <ros/ros.h>
#include <ros/console.h>
#include <array>
#include <stdint.h>

#include <kaimi_utilities/GeoPosition.h>
#include <iostream>

using namespace std;

int debug_last_message = 0; // So that we only emit messages when things change.
		
void logIfChanged(int id, const char* message) {
	if (id != debug_last_message) {
		ROS_INFO_STREAM(message);
		debug_last_message = id;
	}	
}

typedef struct GPS_POINT {
	double	latitude;
	double  longitude;
	GPS_POINT(double lat, double lon) : latitude(lat), longitude(lon) {}
} GPS_POINT;

array<GPS_POINT, 5 /*7*/> pointSet = {
	GPS_POINT(37.35645, -122.0373166)	// NE
	, GPS_POINT(37.356466, -122.0374)	// NW
	//, GPS_POINT(37.3564, -122.0374)		// down 25 feet from NW
	, GPS_POINT(37.35635, -122.0374166)	// down 50 feet from NW
	, GPS_POINT(37.35635, -122.03735)	// down 50 feet from NE
	//, GPS_POINT(37.3564, -122.0373333)  // down 25 feet from NE
	, GPS_POINT(37.35645, -122.0373166)	// NE
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "travel_gps_route_node");

	// ROS node handle.
	ros::NodeHandle nh("~");

	ros::Rate rate(10); // Loop rate

	for (int i = 1; i < 5 /*7*/; i++) {
		GeoPosition from(pointSet[i - 1].latitude, pointSet[i - 1].longitude);
		GeoPosition to(pointSet[i].latitude, pointSet[i].longitude);
		cout << "[" << (i - 1) << "]->[" << i << "] heading: " << to.bearing(from) << ", distance: " << to.distance(from) << " (" << (to.distance(from) * 3.28084) << ")" << endl;
	}

	// while (ros::ok()) {
	// 	try { // Emplement Sequence behavior
	// 		rate.sleep();
	// 		ros::spinOnce();
	// 	} catch(exception& e) {
	// 		ROS_INFO_STREAM("[travel_gps_route_node] Exception: " << e.what());
	// 		// Do nothing.
	// 	}
	// }

	return 0;
}

