#include <array>
#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdint.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include "yaml-cpp/yaml.h"

#include <kaimi_utilities/GeoPosition.h>

using namespace std;

int debug_last_message = 0; // So that we only emit messages when things change.

sensor_msgs::NavSatFix lastFix;
sensor_msgs::Imu lastImu;

long lastFixMessageCount = 0;
long lastImuMessageCount = 0;

string gpsStatus(int8_t status) {
	switch (status) {
		case -1: return "NO FIX";
		case 0: return "FIX";
		case 1: return "FIX W SAT AUG";
		case 2: return "FIX W GND AUG";
		default: return "INVALID VALUE";
	}
}

string gpsService(uint16_t service) {
	string result = "";
	if (service & 1) result.append(" GPS");
	if (service & 2) result.append(" GLONASS");
	if (service & 4) result.append(" COMPASS");
	if (service & 8) result.append(" GALILEO");
	return result;
}

double currentHeadingDegreesAsGps() {
	tf::Quaternion q;
	tf::Matrix3x3 m;
	double imuRoll, imuPitch, imuYaw;
	q = tf::Quaternion(lastImu.orientation.x, lastImu.orientation.y, lastImu.orientation.z, lastImu.orientation.w);
	m = tf::Matrix3x3(q);
	m.getRPY(imuRoll, imuPitch, imuYaw);
	//ROS_INFO("roll: %7.4f, pitch: %7.4f, yaw: %7.4f", imuRoll, imuPitch, imuYaw);
	double result = - GeoPosition::degrees(imuYaw) ; // Magnetic declination
	// if (result < -180) result += 360;
	// if (result > 180) result -= 360;
	return result;
}
		
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	lastImu = *msg;
	lastImuMessageCount++;
}

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	lastFix = *msg;
	lastFixMessageCount++;
}

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

int main(int argc, char** argv) {
	ros::init(argc, argv, "travel_gps_route_node");

	// ROS node handle.
	ros::NodeHandle nh("~");

	ros::Subscriber gpsSub = nh.subscribe("/fix", 1, navSatFixCallback);
	ros::Subscriber imuSUb = nh.subscribe("/kaimi_imu/imu", 1, imuCallback);

	ros::Rate rate(1); // Loop rate

	// Fetch GPS points to move to.
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
	int p = 0;

	//for (auto const& currentDict : gpsConfig["gps_points"]) {
	YAML::const_iterator currentDict = gpsConfig["gps_points"].begin();
	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		YAML::Node n = *currentDict;
		double lat;
		double lon;
		lat = n["latitude"].as<double>();
		lon = n["longitude"].as<double>();
		GeoPosition f(lastFix.latitude, lastFix.longitude);
		GeoPosition t(lat, lon);

		ROS_INFO("[travel_gps_route_node] moving to point: %d"
				 ", CURRENT lat: %11.7f, lon: %11.7f, heading: %7.4f"
				 ", TARGET lat: %11.7f, lon: %11.7f, bearing: %7.4f, distance: %7.4f, GPS: %s-%s",
				 p,
				 lastFix.latitude, lastFix.longitude, currentHeadingDegreesAsGps(),
				 lat, lon, t.bearing(f), t.distance(f),
				 gpsStatus(lastFix.status.status).c_str(), gpsService(lastFix.status.service).c_str()
				 );
		//p++;
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

