#include <ros/ros.h>
#include <ros/console.h>
#include <array>
#include <stdint.h>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <boost/format.hpp>

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

/*typedef*/ struct GPS_POINT {
	double	latitude;
	double	longitude;
	GPS_POINT(double lat, double lon) : latitude(lat), longitude(lon) {}
	GPS_POINT() : latitude(0), longitude(0) {}
}; // GPS_POINT;

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

	YAML::Node gpsConfig = YAML::LoadFile("/home/pi/catkin_ws/src/strategy/config/gps_points.yaml");
	if (!gpsConfig["gps_points"]) {
		cout << "Missing gps_points" << endl;
		return -1;
	}

	if (gpsConfig["gps_points"].Type() != YAML::NodeType::Sequence) {
		cout << "gps_points is not a sequence" << endl;
		return -1;
	}

	GPS_POINT from, to;
	bool needToSkip = true;
	int p = 0;

	for (auto const& currentDict : gpsConfig["gps_points"]) {
		YAML::Node n = currentDict;
		double lat;
		double lon;
		lat = n["latitude"].as<double>();
		lon = n["longitude"].as<double>();
		cout << "lat: " << boost::format("%11.6f") % lat << ", lon: " << boost::format("%11.6f") % lon << endl;
		// for (auto const& element : currentDict) {
		// 	YAML::Node = element;
		// 	string key;
		// 	element->first() >> key;
		// 	double value;
		// 	element->second() >> value;
		// 	cout << "key: " << key << ", value: " << value << endl;
		// }
	}
	// for (YAML::const_iterator it = gpsConfig["gps_pointss"].begin(); it != gpsConfig["gps_pointss"].end(); ++it) {
	// 	//GPS_POINT to = it->as<GPS_POINT>();
	// 	GPS_POINT to;
	// 	//*it >> to;
	// 	for (YAML::const_iterator mit = it->begin(); mit != it->end(); mit++) {
			
	// 		////it->["latitude"] >> to.latitude;
	// 		// string key;
	// 		// double value;
	// 		// mit->first() >> key;
	// 		// mit->second() >> value;
	// 		// cout << "key: " << key << ", value: " << value << endl;
	// 		// if (key == "latitude") {
	// 		// 	to.latitude = value;
	// 		// } else {
	// 		// 	to.longitude = value;
	// 		// }
	// 	}

	// 	if (!needToSkip) {
	// 		GeoPosition f(from.latitude, from.longitude);
	// 		GeoPosition t(to.latitude, to.longitude);
	// 		cout << "2 [" << (p - 1) << "]->[" << p << "] heading: " << t.bearing(f) << ", distance: " << t.distance(f) << " (" << (t.distance(f) * 3.28084) << ")" << endl;
	// 	}

	// 	from = to;
	// 	p++;
	// 	needToSkip = false;
	// 	// GeoPosition from(it->as<vector<int>()[0]gpsPoints[i - 1][.latitude], gpsPoints[i - 1].longitude);
	// 	// GeoPosition to(gpsPoints[i].latitude, gpsPoints[i].longitude);
	// 	// cout << "2 [" << (i - 1) << "]->[" << i << "] heading: " << to.bearing(from) << ", distance: " << to.distance(from) << " (" << (to.distance(from) * 3.28084) << ")" << endl;
	// }

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

