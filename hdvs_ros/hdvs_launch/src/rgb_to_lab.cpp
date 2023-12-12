#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

image_transport::Publisher lab_image_pub;

void camera_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr image_ptr;
	try
	{
		image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		cv::cvtColor(image_ptr->image, image_ptr->image, cv::COLOR_BGR2Lab);

		lab_image_pub.publish(image_ptr->toImageMsg());
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8': %s", msg->encoding.c_str(), e.what());
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rgb_to_lab");
	ros::NodeHandle nh;

	image_transport::ImageTransport camera_it(nh);
	lab_image_pub = camera_it.advertise("/robot_1/camera/image_raw_lab", 1);
  //lab_image_pub = camera_it.advertise("camera/lab/image_raw", 1);
	image_transport::Subscriber camera_sub = camera_it.subscribe("/robot_1/camera/image_raw", 1, camera_callback);
  //image_transport::Subscriber camera_sub = camera_it.subscribe("camera/color/image_raw", 1, camera_callback);

	ros::spin();

	return 0;
}
