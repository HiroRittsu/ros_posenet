#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "json11.hpp"
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"

using namespace std;

cv::Mat depth;
ros::Publisher injected_poses;

void getImage_depth(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	depth = cv_ptr->image;
}

void callback(const std_msgs::String::ConstPtr& msg) {

	string err;
	auto json = json11::Json::parse(msg->data, err);
	ros_posenet::Poses poses;
	printf("%d\n", (int)json["poses"].array_items().size() );
	if (json["poses"].array_items().size() != 0) {
		for (auto &p : json["poses"].array_items()) {

			ros_posenet::Pose pose;
			for (auto &k : p["keypoints"].array_items()) {
				if (std::stod(k["score"].dump()) > 0.5) {

					ros_posenet::Keypoint key;

					key.position.x = std::stod(k["position"]["x"].dump());
					key.position.y = std::stod(k["position"]["y"].dump());
					key.position.z = depth.at<double>((int)key.position.y, (int)key.position.x);
					key.score = std::stod(k["score"].dump());
					key.part = k["part"].dump();

					pose.keypoints.push_back(key);

					//cv::circle(depth, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
				}
			}
			poses.poses.push_back(pose);
		}
		//cv::imshow("Color", depth );
		//cv::waitKey(1);
		injected_poses.publish(poses);
	} else {
		printf("error\n");
	}
}


int main(int argc, char **argv) {

	ROS_INFO("debug#############################");

	ros::init(argc, argv, "injection");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/js_poses", 1000, callback);
	ros::Subscriber left_sub = n.subscribe("/openni2/depth", 1, getImage_depth);
	injected_poses = n.advertise<ros_posenet::Poses>("/ros_posenet/poses", 1000);

	ros::spin();

	return 0;
}