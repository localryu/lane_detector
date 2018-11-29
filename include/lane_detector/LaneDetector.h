#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

/**
*@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
*@brief Activity diagram and UML Class diagram.
*@brief It detects the lanes in an image if a highway and outputs the
*@brief same image with the plotted lane.
*/
class LaneDetector {
private:
	double img_size;
	double img_center;
	bool left_flag = false;  // Tells us if there's left boundary of lane detected
	bool right_flag = false;  // Tells us if there's right boundary of lane detected
	cv::Point right_b;  // Members of both line equations of the lane boundaries:
	double right_m;  // y = m*x + b
	cv::Point left_b;  //
	double left_m;  //

	//HoughLinesP variables
	uchar LINE_LENGTH = 30;

	//regression variables
	double detect_n = 0.30; // detection point(line) of y axis for line regression(also apply to visualization).(the percentage of image column)
 	
	// ROI variables
	float height1 = 0.3;
	float height2 = 0.5;
	float height3 = 1;
	float x1 = 0.3;
	float x2 = 0;
	float x3 = 0;

	//color filtering variables
	cv::Scalar lower_white_rgb; //��� ���� (RGB)
	cv::Scalar upper_white_rgb;
	cv::Scalar lower_yellow_hsv; //����� ���� (HSV)
	cv::Scalar upper_yellow_hsv;
	cv::Scalar lower_white_hsv; //��� ���� (RGB)
	cv::Scalar upper_white_hsv;
 	uchar WITHE_RGB_THRES = 200;

public:
	cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
	cv::Mat mask(cv::Mat img_edges);  // Mask the edges image to only care about ROI
	void filter_colors(cv::Mat _img_bgr, cv::Mat &img_filtered);
	double steer_control(cv::Mat denoise, int height_percent, int judging_line, cv::Mat frame, int &zero_count);

};

#endif
