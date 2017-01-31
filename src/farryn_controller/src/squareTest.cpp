#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "std_msgs/String.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "square_tester");
	ros::NodeHandle rosNode;
	ros::Publisher cmdvelPublisher = rosNode.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	ros::Rate r(0.5);

	geometry_msgs::Twist cmdVel;

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.5;
	cmdVel.angular.z = 0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();


	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = M_PI / 2.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();


	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.5;
	cmdVel.angular.z = 0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();


	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = M_PI / 2.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.5;
	cmdVel.angular.z = 0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = M_PI / 2.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.5;
	cmdVel.angular.z = 0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();


	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = M_PI / 2.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	r.sleep();

	return 0;
}