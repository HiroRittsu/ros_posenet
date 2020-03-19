#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include <unordered_set>
#include "json11.hpp"
#include "ros_posenet/Poses.h"
#include <limits>


using namespace std;

cv::Mat color;

bool is_point(cv::Point3f point) {

	bool flag = false;

	if (isnan(point.x) || point.x == 0) flag = true;
	if (isnan(point.y) || point.y == 0) flag = true;
	if (isnan(point.z) || point.z == 0) flag = true;

	return flag;
}

double distance(cv::Point3f first, cv::Point3f end) {

	double X = end.x - first.x;
	double Y = end.y - first.y;
	double Z = end.z - first.z;

	return sqrt(X * X + Y * Y + Z * Z);
}

std::vector<cv::Point3f> clustering(std::vector<cv::Point3f> centroids) {

	int reference_index = 0;

	std::vector<cv::Point3f> target_points;

	while (1) {

		//deep copy
		target_points = centroids;

		//nan
		if (is_point(target_points[reference_index])) {
			reference_index ++;
			if (reference_index >= (int)target_points.size()) break;
			continue;
		}

		std::unordered_set<int> fix_index;
		std::unordered_set<int> deposit_index;

		bool fixed_flag = false;

		for (int j = 0; j < (int)target_points.size(); ++j) {
			if (is_point(target_points[j])) continue;

			//二点間の距離
			if (reference_index != j) {
				//近い点だった場合
				if (distance(target_points[reference_index], target_points[j]) < 300) {
					fix_index.emplace(j);
					fix_index.emplace(reference_index);
					fixed_flag = true;
				} else {
					deposit_index.emplace(j);
				}
			}
		}

		if (!fixed_flag) {
			reference_index++;
			if (reference_index >= (int)target_points.size()) break;
			continue;
		}

		//最適化
		centroids.clear();
		cv::Point3f result{0, 0, 0};
		for (auto i : fix_index) {
			result.x += target_points[i].x;
			result.y += target_points[i].y;
			result.z += target_points[i].z;
		}
		//重心計算
		result.x = result.x / (int)(fix_index.size());
		result.y = result.y / (int)(fix_index.size());
		result.z = result.z / (int)(fix_index.size());
		centroids.push_back(result);

		for (auto i : deposit_index) {
			centroids.push_back(target_points[i]);
		}
		
		reference_index = 0;
	}

	return target_points;
}

void getImage_color(const sensor_msgs::Image::ConstPtr & msg) {

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

void poses(const ros_posenet::Poses::ConstPtr & msg) {

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
			//cv::circle(color, cv::Point(centroid.x, centroid.y), 30, cv::Scalar(255, 0, 0), -1);
		}

		cout << centroids << '\n';

		centroids = clustering(centroids);

		for (auto centor : centroids) {
			cv::circle(color, cv::Point(centor.x, centor.y), 30, cv::Scalar(255, 0, 0), -1);
		}

		cv::imshow("Color", color);
		cv::waitKey(1);
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "pose_centroid");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/ros_posenet/poses", 1000, poses);
	ros::Subscriber left_sub = n.subscribe("/openni2/color", 1, getImage_color);


	ros::spin();

	return 0;
}