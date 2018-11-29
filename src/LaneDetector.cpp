#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lane_detector/LaneDetector.h"


using namespace std;
using namespace cv;

#define PI 3.141592

//bluring the images for remove noise.
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
	cv::Mat output;
	cv::medianBlur(inputImage, output, 3);
	return output;
}


// extract white and yellow lane.
void LaneDetector::filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	lower_white_rgb = Scalar(WITHE_RGB_THRES, WITHE_RGB_THRES, WITHE_RGB_THRES); //(RGB)
	upper_white_rgb = Scalar(255, 255, 255);
	lower_yellow_hsv = Scalar(10, 180, 180); // (HSV)
	upper_yellow_hsv = Scalar(40, 255, 255);
	lower_white_hsv = Scalar(0, 0, 180); //(HSV)
	upper_white_hsv = Scalar(360, 65, 255);
	// Filter the image to include only yellow and white pixels
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);
	Mat test;
	_img_bgr.copyTo(test);
	Mat img_hsv, img_combine;
	Mat white_mask_rgb, white_image_rgb;
	Mat yellow_mask, yellow_image;
	Mat white_mask_hsv, white_image_hsv;
	Mat thre, grayy;

	cvtColor(img_bgr, grayy, COLOR_BGR2GRAY);
	cv::threshold(grayy, thre, 160, 255, cv::THRESH_BINARY);
	imshow("thre....", thre);

	/*
	//Filter white pixels with RGB
	inRange(img_bgr, lower_white_rgb, upper_white_rgb, white_mask_rgb);
	bitwise_and(img_bgr, img_bgr, white_image_rgb, white_mask_rgb);
	imshow("white rgb", white_image_rgb);

	//using white - hsv filtering
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);
	inRange(img_hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);
	bitwise_and(img_bgr, img_bgr, white_image_hsv, white_mask_hsv);

	cvtColor(white_image_hsv,white_image_hsv,COLOR_BGR2GRAY);
	cv::threshold(white_image_hsv, white_image_hsv, 170, 255, cv::THRESH_BINARY);
	//imshow("white_hsv", white_image_hsv);
	*/

	thre.copyTo(img_filtered);


}

// MASK THE EDGE IMAGE
/**
 *@brief Mask the image so that only the edges that form part of the lane are detected
 *@param img_edges is the edges image from the previous function
 *@return Binary image with only the desired edges being represented
 */


cv::Mat LaneDetector::mask(cv::Mat frame) {
	cv::Mat output;
	cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());
	cv::Mat mask2 = cv::Mat::zeros(frame.size(), frame.type());

	cv::Point pts[4] = {
		cv::Point(0, frame.rows),
		cv::Point(0, frame.rows/2),
		cv::Point(frame.cols, frame.rows / 2),
		cv::Point(frame.cols, frame.rows)
	};
	// Create a binary polygon mask
	cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 255, 255));

	// Multiply the edges image and the mask to get the output
	cv::bitwise_and(frame, mask, output);

	return output;
}




// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param lane is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */
 double LaneDetector::steer_control(Mat denoise, int height_percent, int judging_line, Mat frame, int &zero_count)
 {
 	int left_x_num = 0;
 	int right_x_num = 0;
 	int left_sum_x = 0;
 	int right_sum_x = 0;
	int left_x =0;
	int left_x_buf = 0;
	int right_x =frame.cols;
	int right_x_buf = frame.cols;

 	int line_height = denoise.rows * height_percent / 100.0;

 	for (int j = line_height ; j < line_height + judging_line; j++)
 	{
 		for (int i = 2; i < denoise.cols / 3.0; i++)
 		{

 			//left lane.
 			if (denoise.at<uchar>(j, i) == 255)
 			{
 				left_sum_x += i;
 				left_x_num++;

 			}

 			//right lane
 			if (denoise.at<uchar>(j, denoise.cols - i) == 255)
 			{
 				right_sum_x += denoise.cols - i;
 				right_x_num++;
 			}
 		}
 	}
    
 	line(denoise, Point(2, line_height), Point(denoise.cols, line_height), Scalar(0, 255, 255), 5);
 	line(denoise, Point(denoise.cols - 2, line_height), Point(denoise.cols * 2 / 3.0, line_height), Scalar(255, 255, 255), 5);

 	if (left_x_num > 100)
 	{
 		left_x = left_sum_x / left_x_num;
		left_x_buf = left_x;
 	}
	else{
		left_x = left_x_buf;
	}
 	if (right_x_num > 100)
 	{
 		right_x = right_sum_x / right_x_num;
		right_x_buf = right_x;
 	}
	else{
		right_x = right_x_buf;
	}

	if((left_x_num == 0) && (right_x_num == 0)){
		zero_count ++;
	}else{
		zero_count = 0;
	}

 	int middle = (left_x + right_x) / 2.0;
 	double angle = atan2(middle - denoise.cols / 2, denoise.rows - line_height) * 180 / PI;


 	// plot
 	line(frame, Point(2, line_height), Point(frame.cols / 3.0, line_height), Scalar(0, 255, 255), 5);
 	line(frame, Point(frame.cols - 2, line_height), Point(frame.cols * 2 / 3.0, line_height), Scalar(255, 255, 255), 5);
 	//circle(frame, Point(frame.cols/2, (frame.rows/2), 5, Scalar(255, 0, 0), 5);
 	circle(frame, Point(left_x, line_height), 5, Scalar(255, 0, 0), 5);
 	circle(frame, Point(right_x, line_height), 5, Scalar(255, 0, 0), 5);
	line(frame, Point(frame.cols/2, 0), Point(frame.cols/2, frame.rows), Scalar(255,0,0), 1 );
 	line(frame, Point(middle, line_height), Point(denoise.cols / 2, denoise.rows), Scalar(255, 255, 255), 4);
 	imshow("frame", frame);
	waitKey(3);

	return angle;
 }
