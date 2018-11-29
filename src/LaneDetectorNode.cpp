#include "lane_detector/LaneDetectorNode.h"

using namespace std;
using namespace cv;

LaneDetectorNode::LaneDetectorNode()
{
	nh = ros::NodeHandle();
	nh_ = ros::NodeHandle("~");

	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
	control_pub_ = nh.advertise<std_msgs::String>("write", 1000);
	//control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &LaneDetectorNode::imageCallback, this);

	getRosParamForUpdate();
}

//LaneDetectorNode::LaneDetectorNode(String path)
//	: test_video_path(path)
//{}


void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{	
	bool lane_status = true;
	try{
		parseRawimg(image, frame);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}

	getRosParamForUpdate();

	steer_control_value_ = laneDetecting();
	
	std_msgs::String control_msg = makeControlMsg(steer_control_value_);
	//ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
	
	cout << "steer : " << steer_control_value_ << "  throttle : " << throttle_ << endl;
	control_pub_.publish(control_msg);
}


void LaneDetectorNode::getRosParamForUpdate()
{
	nh_.getParam("throttle", throttle_);
	nh_.getParam("angle_factor", angle_factor_);
	nh_.getParam("stop_count", stop_count);
	nh_.getParam("steer_height", steer_height);
}

std_msgs::String LaneDetectorNode::makeControlMsg(int steer)
{
	std_msgs::String control_msg;
	control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
	return control_msg;
}

/*
ackermann_msgs::AckermannDriveStamped LaneDetectorNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
}
*/

int LaneDetectorNode::laneDetecting()
{
	int ncols = frame.cols;
	int nrows = frame.rows;
	double angle_ideal = 0;
	double angle_ = 0;
	
	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));
	img_mask = lanedetector.mask(lane_frame);
	lanedetector.filter_colors(img_mask, img_mask2);
	img_denoise = lanedetector.deNoise(img_mask2);

	double angle = lanedetector.steer_control(img_denoise, steer_height, 12, img_mask, zero_count);
	ROS_INFO("zero_count: %d", zero_count);
	if(zero_count > stop_count){
		mission_cleared= true;
		return 0;
	}
	else{
		mission_cleared = false;
	}

	angle_ideal = angle * angle_factor_;
	angle_ = STEER_ZERO + angle_ideal;
	//for limit platform angle values.	

	if(angle_ > STEER_ZERO + MAX_STEER){
		angle_ = STEER_ZERO + MAX_STEER;
	}
	else if (angle_ < STEER_ZERO - MAX_STEER){
		angle_ = STEER_ZERO - MAX_STEER;
	}
	

	return angle_;
}



void LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}


