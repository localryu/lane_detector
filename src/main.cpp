#include "ros/ros.h"
#include <iostream>
#include "lane_detector/LaneDetectorNode.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");
	// using camera
	LaneDetectorNode lane_detector_node;

	ros::spin();

	return 0;
}
