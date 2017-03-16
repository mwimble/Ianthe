#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "MPU6050.h"

void setup_covariance(boost::array<double, 9> &cov, double stdev) {
    std::fill(cov.begin(), cov.end(), 0.0);
    if (stdev == 0.0)
        cov[0] = -1.0;
    else {
        cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
    }
}

int main(int argc, char** argv) {
	boost::array<double, 9> linear_acceleration_cov = {-1,0,0,0,0,0,0,0,0};
	boost::array<double, 9> angular_velocity_cov = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
	boost::array<double, 9> orientation_cov = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

    // setup_covariance(linear_acceleration_cov, 0.0003);
    // setup_covariance(angular_velocity_cov, 0.02 * (M_PI / 180.0));
    // setup_covariance(orientation_cov, 1.0);

	ros::init(argc, argv, "kaimi_imu_node");
	ros::NodeHandle nh("~");
	ros::Publisher imuPub;

	bool showImu;
	nh.getParam("showImu", showImu);
	ROS_INFO("[kaimi_imu_node] showImu: %s", showImu ? "TRUE" : "false");
	imuPub = nh.advertise<sensor_msgs::Imu>("imu", 20);
	ros::Rate rate(100); // Loop rate

	MPU6050 mpu;

    ROS_INFO("[kaimi_imu_node] Initializing I2C devices");
    mpu.initialize();

    ROS_INFO("[kaimi_imu_node] Testing device connections...\n");
    bool mpuConnectionOk = mpu.testConnection();
    if (!mpuConnectionOk) {
    	ROS_FATAL("[kaimi_imu_node] Unable to connect to MPU6050");
    	return -1;
    } else {
    	ROS_INFO("[kaimi_imu_node] IMU connection successful");
    }

    ROS_INFO("[kaimi_imu_node] Initializing the DMP");
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
    	ROS_FATAL("[kaimi_imu_node] Unable to initialize the DMP");
    	return -1;
    } else {
    	ROS_INFO("[kaimi_imu_node] DMP successfully initialized");
    }

    ROS_INFO("[kaimi_imu_node] Enabling the DMP");
    mpu.setDMPEnabled(true);

    uint8_t mpuIntStatus = mpu.getIntStatus();
    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.setSlave2FIFOEnabled(true);//###
    ROS_INFO("[kaimi_imu_node] INT status: %d, packetSize: %d", mpuIntStatus, packetSize);

    uint8_t fifoBuffer[64];
    Quaternion q;
    sensor_msgs::Imu imu = sensor_msgs::Imu();
    VectorInt16 aa; 		// Accelerometer sensor measurements
    VectorInt16 aaReal;		// Gravity-free accelerometer sensor measurements.
    VectorInt16 aaWorld;		// Gravity-free accelerometer sensor measurements.
    VectorFloat gravity;	// Gravity vector.
    int16_t gyro[3];		// Gyroscope, raw.

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		uint16_t fifoCount = mpu.getFIFOCount();
		if (fifoCount == 1024) {
			// Reset so we can continue cleanly.
			mpu.resetFIFO();
			//ROS_INFO("[kaimi_imu_node] FIFO overflow!");
		} else if (fifoCount >= 42) {
			imu.header.frame_id = "imu";
			imu.header.stamp =ros::Time::now();
			// Read a packet from the FIFO.
		    imu.orientation_covariance = orientation_cov;
		    imu.angular_velocity_covariance = angular_velocity_cov;
		    imu.linear_acceleration_covariance = linear_acceleration_cov;

			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			imu.orientation.w = q.w;
			imu.orientation.x = q.x;
			imu.orientation.y = q.y;
			imu.orientation.z = q.z;
			//ROS_INFO("Quaternion w: %7.2f, x: %7.2f, y: %7.2f, z: %7.2f", q.w, q.x, q.y, q.z);

			int16_t v[9];
			mpu.getMotion9(&v[0], &v[1], &v[2], &v[3], &v[5], &v[5], &v[6], &v[7], &v[8]);
			ROS_INFO("[kaimi_imu_node] v0: %d, v1: %d, v2: %d, v3: %d, v4: %d, v5: %d, v6: %d, v7: %d, v8: %d", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
			float ypr[3];
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//			ROS_INFO("ypr  %7.2f %7.2f %7.2f  || w: %f, z: %f, y: %f, x: %f  ", 
//					  ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI,
//					  imu.orientation.w, imu.orientation.z, imu.orientation.y, imu.orientation.x);

			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			imu.linear_acceleration.x = aaWorld.x;
			imu.linear_acceleration.y = aaWorld.y;
			imu.linear_acceleration.z = aaWorld.z;

			mpu.dmpGetGyro(&gyro[0], fifoBuffer);
			imu.angular_velocity.x = gyro[0] * 25.0;
			imu.angular_velocity.y = gyro[1] * 25.0;
			imu.angular_velocity.z = gyro[2] * 25.0;

			//ROS_INFO("gyro x: %d, y: %d, z: %d -- %f, %f, %f", gyro[0], gyro[1], gyro[2], imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
			imuPub.publish(imu);
			if (showImu) {	
				tf::Quaternion qq;
				double roll, pitch, yaw;
				tf::quaternionMsgToTF(imu.orientation, qq);
				tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
				ROS_INFO("[kaimi_imu_node] roll: %7.2f, pitch: %7.2f, yaw: %7.2f", roll, pitch, yaw);

			}
		}
	}
}
