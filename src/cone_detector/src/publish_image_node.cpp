#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_image_node");
    ros::NodeHandle nh;
    std::string image_path;
    std::string image_topic_name;

    ros::param::get("~image_path", image_path);
    ros::param::get("~image_topic_name", image_topic_name);
    ROS_INFO("[publish_image_node] image_path: %s", image_path.c_str());
    ROS_INFO("[publish_image_node] image_topic_name: %s", image_topic_name.c_str());
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(image_topic_name, 1);
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);
    if (image.empty()) {
        ROS_INFO("[publish_image_node] unable to load image: \"%s\"", image_path.c_str());
        exit(-1);
    }

    ROS_INFO("[publish_image_node] image size cols: %d, rows: %d", image.cols, image.rows);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}