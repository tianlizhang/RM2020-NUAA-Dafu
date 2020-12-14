#ifndef MARKERSSENSOR_H
#define MARKERSSENSOR_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/tracking.hpp>
//#include "templeteTracking.h"
#include<memory>
#include "MarkerParams.h"
//#include "utils.h"
//#include "tf/tf.h"
//timer
//#include "timer.h"
using namespace std;
using namespace cv;

class RotRect {
public:
	cv::Point2f center;
	cv::Point2f dir1,dir2;
	float width;
	float height;
	RotRect() :width(0), height(0) {};
	RotRect(const cv::Rect & rect) :width(rect.width), height(rect.height) {
		center.x = rect.x + rect.width*0.5f;
		center.y = rect.y + rect.height*0.5f;
		dir1.x = 1; dir1.y = 0;
		dir2.x = 0; dir2.y = 0;
	}
	int Draw(Mat & img) {
		//int is_dir0_down = (dir.dot(Point2f(0, 1)))>0 ? 1 : -1;
		//int is_dir1_down = (dir.dot(Point2f(0, 1)))>0 ? 1 : -1;
		//Point2f   kpts[4];
		//kpts[0] = center - is_dir0_down*dir*width*0.5f;
		//kpts[2] = center + is_dir0_down*dir*width*0.5f;
		//kpts[1] = center - is_dir1_down*dir*width*0.5f;
		//kpts[3] = center + is_dir1_down*dir*width*0.5f;
		/*cv::line(img, kpts[0], kpts[1], cv::Scalar(255, 0, 0), 3);
		cv::line(img, kpts[1], kpts[2], cv::Scalar(0, 255, 0), 3);
		cv::line(img, kpts[2], kpts[3], cv::Scalar(0, 0, 255), 3);
		cv::line(img, kpts[3], kpts[0], cv::Scalar(255, 255, 0), 3);*/
		circle(img, center, 4, Scalar(255, 255, 255));
		line(img, center, center+100.0*dir2, Scalar(255, 255, 255), 2);
		return 0;
	}
};
class Marker {
public:
	Marker() {
		old_depth = 0;
		depth = 0;
	}
	enum MarkerType {
		All = 0,
		SMALL = 1,
		BIG = 2
	};
	int ComputeKeyPoints() {
		int is_dir0_down = (LEDs[0].dir1.dot(Point2f(0, 1)))>0 ? 1 : -1;
		int is_dir1_down = (LEDs[1].dir1.dot(Point2f(0, 1)))>0 ? 1 : -1;
		kpts[0] = LEDs[0].center - is_dir0_down*LEDs[0].dir1*LEDs[0].width*0.5f;
		kpts[3] = LEDs[0].center + is_dir0_down*LEDs[0].dir1*LEDs[0].width*0.5f;
		kpts[1] = LEDs[1].center - is_dir1_down*LEDs[1].dir1*LEDs[1].width*0.5f;
		kpts[2] = LEDs[1].center + is_dir1_down*LEDs[1].dir1*LEDs[1].width*0.5f;
		return 0;
	}
	int ComputeBBox() {
		float max_x = 0, max_y = 0;
		float min_x = 9999, min_y = 9999;
		for (int i = 0; i < 4; i++) {
			Point2f kpt = kpts[i];			// may be wrong
			if (kpt.x < min_x) {
				min_x = kpt.x;
			}
			if (kpt.x > max_x) {
				max_x = kpt.x;
			}
			if (kpt.y < min_y) {
				min_y = kpt.y;
			}
			if (kpt.y > max_y) {
				max_y = kpt.y;
			}
		}
		bbox.x = min_x;
		bbox.y = min_y;
		bbox.width = (max_x - min_x);
		bbox.height = (max_y - min_y);
		return 0;
	}
	int Draw(Mat & img) {
		ComputeKeyPoints();
		cv::line(img, kpts[0], kpts[1], cv::Scalar(255, 0, 0), 2);
		cv::line(img, kpts[1], kpts[2], cv::Scalar(255, 0, 0), 2);
		cv::line(img, kpts[2], kpts[3], cv::Scalar(255, 0, 0), 2);
		cv::line(img, kpts[3], kpts[0], cv::Scalar(255, 0, 0), 2);
		return 0;
	}
	RotRect   LEDs[2];
	Point2f   kpts[4];
	Rect      bbox;
	//float decision_points;
	float   old_depth, depth;
	//MarkerType armor_type;
};
class MarkSensor {
public:
	enum SensorStatus {
		STATUS_SUCCESS = 0,
		STATUS_TRACKING,
		STATUS_TRACKLOST0,
		STATUS_TRACKLOST1,
		STATUS_TRACKLOST2,
		STATUS_DETECTING
	};
	enum EnemyStatus {
		STATIC_POS = 0,
		MOVING = 1,
		SWAGGING = 2,
		ROTATING = 3
	};
	//AlgoriParam ap;
	//CamParams cp;
	MarkerParams mp;
	//Mat cameraMatrix,distCoeffs;
	SensorStatus status = STATUS_DETECTING;
	Marker marker;
	Mat img_gray, img_bgr, img_hsv, img_h, led_mask, img_out;
	//static 
		Mat img_show, ROI_bgr;
	//Point2f old_target, target;
	int track_fail_cnt[3];
	//tf::Transform trans;
	//bool got_trans = false;
	//tf::Vector3 pos_t2w;
	//tf::Vector3 pos_t2c;
	//for motion analyse
	//EnemyStatus enemy_stat;
	//deque<float> leaky_list;
	MarkSensor(MarkerParams &mp_,CamParams &cp_) : mp(mp_){
		//cameraMatrix = (cv::Mat_<double>(3, 3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
		//distCoeffs = (Mat_<double>(1, 4) << cp.distcoef1, cp.distcoef2, 0, 0);
	}
	int focus2Lines(Point2f dir1,Point2f pt1,Point2f dir2,Point2f pt2,Point2f &res);
	int MarkSensor::chooseMarker(vector<Marker> markers);
	int MarkSensor::detectTrackFrame(const Mat &img);
	int DetectMarker(const Mat &img, Marker &res_marker);
	int TrackMarker(const Mat &img, Marker &res_marker);
	int GetMarker(cv::Mat &roi_mask, Marker &res_marker);
	int PCAMarker(vector<cv::Point> &contour, Point2f &dir1, Point2f &dir2, Point2f &center);
	float ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir);
	int MarkSensor::getCircleCenter(Point2f &circle_center, Marker &res_marker, vector <RotRect> boxs);
	Point2f MarkSensor::predictObject(int preTheta, Point2f &circle_center, Marker &marker);
	//计算云台pitch轴转角，若目标点在相机坐标系下P(x,y,z)
	// HorizontalDistance 摄像头距离大符的水平距离 =sqrt(x*x+z*z)
	// horizontal_diff	  目标打击点距离相机水平距离 =x
	// height_diff		  目标打击点高度-摄像头高度 =abs(y)
	// PitchDegree        云台仰角 单位度 抬头为正
	// BulletVelocity     子弹飞行速度
	int getPitchYawDegree(float horizontal_dist, float horizontal_diff,float height_diff, float BulletVelocit, float &PitchDegree, float &YawDegree);
	/*int paraDistance(RotRect &LED1, RotRect &LED2);
	int Kalman();
	int tgt_selector(vector<Marker> &markers);
	int GammaCorrect();
	int calcDepth(Marker &marker);
	int judge_motion();*/
	
};

//class HaarD
//{
//public:
//	bool Detect_track(const Mat & img, float & X, float & Y, float & Z, int &type, int &pix_x, int &pix_y);
//	vector<Rect> color_filter(Mat frame, vector<Rect> boards, bool color_flag);	//color filter
//	bool judge_color(Mat src);
//	//HaarD(String cascade_name= "zgdcascade_1.xml");
//	//TemplateTracker tracker;
//	int status = 0;
//	int frame_num = 0;
//	CascadeClassifier detector;
//	bool show_visualization = 1;
//	Rect location;
//	bool color_flag = 1;  // false: blue   true : red
//private:
//};
//void limitRect(Rect &location, Size sz);
#endif
