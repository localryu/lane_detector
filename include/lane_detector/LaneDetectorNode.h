#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lane_detector/LaneDetector.h"
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>


#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

#define resize_n 1 // variable for image(frame) resize
// method 1 :

class LaneDetectorNode
{
	public:
		LaneDetectorNode();

		LaneDetectorNode(cv::String path);

		/**
		 * Run test that use video file.
		 */
		bool run_test();

		/**
		 * @brief 카메라로부터 들어온 이미지를 Subscribe 했을 때 호출되는 Callback 함수
		 *
		 * @param image 카메라 드라이버 노드에서 보낸 이미지를 받아 포인팅하고있는 포인터
		 *
		 */
		void imageCallback(const sensor_msgs::ImageConstPtr& image);
		//void actionCallback(const state_cpp_msg::MissionPlannerGoalConstPtr& goal);

	protected:
		/**
		 * @brief 차선 인식과 관련된 파라미터 중 동적으로 바뀔 수 있는 값들을 읽어오는 함수
		 *
		 * @details 이 함수는 주기적으로 계속 호출되므로, rosparam을 통해 노드 실행중 동적으로 값들을 바꾸면서 테스트가 가능하다.
		 */
		void getRosParamForUpdate();

		/**
		 * @brief Ros 통신에서 사용하는 이미지 타입을 Opencv의 Mat 타입으로 변환해주는 함수
		 *
		 */
		void parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);

		/**
		 * @brief make control message
		 *
		 */
		//ackermann_msgs::AckermannDriveStamped makeControlMsg();
		std_msgs::String makeControlMsg(int steer);

		/**
		 * @brief lane detecting wrapper
		 *
		 */
		int laneDetecting();


	protected:
		ros::NodeHandle nh;
		ros::NodeHandle nh_;
		ros::Publisher control_pub_;	// Controll message Publisher
		ros::Subscriber image_sub_;	//raw image message Subscriber
		//actionlib::SimpleActionServer<state_cpp_msg::MissionPlannerAction> as_;

		bool mission_start = false;
		bool mission_cleared = false;

		int zero_count = 0;
		int stop_count = 100;

		int throttle_ = 0;
		int steer_control_value_= 0;
		double angle_factor_ = 0.7;
		int steer_height = 70;
		bool u_turn;

		LaneDetector lanedetector;  // Create the class object
		cv::Mat frame;
		cv::Mat lane_frame;
		cv::Mat img_denoise;
		cv::Mat img_edges;
		cv::Mat img_mask;
		cv::Mat img_lines;
		cv::Mat img_mask2;
		cv::Mat img_mask3;
		std::vector<cv::Vec4i> lines;
		std::vector<std::vector<cv::Vec4i> > left_right_lines;
		std::vector<cv::Point> lane;
		std::string turn;
		int flag_plot = -1;
		int i = 0;
		double avg = 0;
		double sum = 0;
		int frame_count = 0;
		int j = 0;
		double angle = 0;
		int MAX_STEER = 260;
		int STEER_ZERO = 1450;

};

#endif
