#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "json11.hpp"
#include "ros_posenet/Poses.h"

using namespace std;

cv::Mat color;

void getImage_color(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	color = cv_ptr->image;
}

void view(const ros_posenet::Poses::ConstPtr& msg) {

	std::vector<cv::Point3f> centroids;

	if (!color.empty()) {
		for (int i = 0; i < (int)msg->poses.size(); ++i) {
			cv::Point3f centroid{0, 0, 0};
			for (auto k : msg->poses[i].keypoints) {
				if (k.position.z != 0) {
					cv::circle(color, cv::Point(k.position.x, k.position.y), 3, cv::Scalar(0, 0, 255), -1);
					cv::putText(color, to_string(k.position.z) , cv::Point(k.position.x - 50, k.position.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2, CV_AA);
					//座標加算
					centroid.x += k.position.x;
					centroid.y += k.position.y;
					centroid.z += k.position.z;
				}
			}
			//重心計算
			centroid.x = centroid.x / (int)(msg->poses[i].keypoints.size());
			centroid.y = centroid.y / (int)(msg->poses[i].keypoints.size());
			centroid.z = centroid.z / (int)(msg->poses[i].keypoints.size());
			centroids.push_back(centroid);
			cv::circle(color, cv::Point(centroid.x, centroid.y), 30, cv::Scalar(255, 0, 0), -1);

		}
		printf("debug\n");
		cout << centroids << '\n';

		cv::imshow("Color", color);
		cv::waitKey(1);
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "viewer");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/ros_posenet/poses", 1000, view);
	ros::Subscriber left_sub = n.subscribe("/openni2/color", 1, getImage_color);


	ros::spin();

	return 0;
}