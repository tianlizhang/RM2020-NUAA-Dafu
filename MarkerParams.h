#ifndef MARKERPARAMS_H
#define MARKERPARAMS_H
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
//#include <ros/ros.h>
using namespace std;
#define RAD2ANG 57.32  //1弧度=57.32度

class CamParams {
public:
	//ros::NodeHandle nh_;
	int rows, cols, fps;
	float cx, cy, fx, fy, distcoef1, distcoef2;
	CamParams(int rows_, int cols_, int fps_,
		float cx_, float cy_,
		float fx_, float fy_,
		float distcoef1_, float distcoef2_) :
		rows(rows_), cols(cols_),
		cx(cx_), cy(cy_),
		fx(fx_), fy(fy_),
		fps(fps_), distcoef1(distcoef1_), distcoef2(distcoef2_)
	{}
	CamParams(int cam_idx, bool is_large) {
		if (is_large) {
			if (cam_idx == 1) {
				/*nh_.getParam("/CameraParams_1_big/rows", rows);
				nh_.getParam("/CameraParams_1_big/cols", cols);
				nh_.getParam("/CameraParams_1_big/fps", fps);
				nh_.getParam("/CameraParams_1_big/cx", cx);
				nh_.getParam("/CameraParams_1_big/cy", cy);
				nh_.getParam("/CameraParams_1_big/fx", fx);
				nh_.getParam("/CameraParams_1_big/fy", fy);
				nh_.getParam("/CameraParams_1_big/distcoef1", distcoef1);
				nh_.getParam("/CameraParams_1_big/distcoef2", distcoef2);*/
			}
			else if (cam_idx == 2) {
				/*nh_.getParam("/CameraParams_2_big/rows", rows);
				nh_.getParam("/CameraParams_2_big/cols", cols);
				nh_.getParam("/CameraParams_2_big/fps", fps);
				nh_.getParam("/CameraParams_2_big/cx", cx);
				nh_.getParam("/CameraParams_2_big/cy", cy);
				nh_.getParam("/CameraParams_2_big/fx", fx);
				nh_.getParam("/CameraParams_2_big/fy", fy);
				nh_.getParam("/CameraParams_2_big/distcoef1", distcoef1);
				nh_.getParam("/CameraParams_2_big/distcoef2", distcoef2);*/
			}
		}
		else {
			if (cam_idx == 1) {
				/*nh_.getParam("/CameraParams_1_small/rows", rows);
				nh_.getParam("/CameraParams_1_small/cols", cols);
				nh_.getParam("/CameraParams_1_small/fps", fps);
				nh_.getParam("/CameraParams_1_small/cx", cx);
				nh_.getParam("/CameraParams_1_small/cy", cy);
				nh_.getParam("/CameraParams_1_small/fx", fx);
				nh_.getParam("/CameraParams_1_small/fy", fy);
				nh_.getParam("/CameraParams_1_small/distcoef1", distcoef1);
				nh_.getParam("/CameraParams_1_small/distcoef2", distcoef2);*/
			}
			else if (cam_idx == 2) {
				/*nh_.getParam("/CameraParams_2_small/rows", rows);
				nh_.getParam("/CameraParams_2_small/cols", cols);
				nh_.getParam("/CameraParams_2_small/fps", fps);
				nh_.getParam("/CameraParams_2_small/cx", cx);
				nh_.getParam("/CameraParams_2_small/cy", cy);
				nh_.getParam("/CameraParams_2_small/fx", fx);
				nh_.getParam("/CameraParams_2_small/fy", fy);
				nh_.getParam("/CameraParams_2_small/distcoef1", distcoef1);
				nh_.getParam("/CameraParams_2_small/distcoef2", distcoef2);*/
			}
		}
	}
};
class MarkerParams {
public:
	//ros::NodeHandle nh_;
	bool is_red;
	int  ch1_min, ch1_max, ch2_min, ch2_max, ch3_min, ch3_max;
	string dbg_path;
	// 一个轮廓筛选
	int contour_direction_angle_min, contour_direction_angle_max; // 轮廓方向[0,pi],与x轴的下方夹角
	// 第一个轮廓
	int   contour_lengtch1_min, contour_lengtch1_max; //轮廓长度
	int contour_widtch1_min, contour_widtch1_max; //轮廓宽度
	int contour_ratio1_min, contour_ratio1_max; //轮廓比例
	// 第二个轮廓
	int   contour_lengtch2_min, contour_lengtch2_max; //轮廓长度
	int contour_widtch2_min, contour_widtch2_max; //轮廓宽度
	int contour_ratio2_min, contour_ratio2_max; //轮廓比例
	// 两个轮廓之间筛选
	int contours_dist_min, contours_dist_max; //两个轮廓之间距离
	int contours_c2c_angle_max; //两轮廓中心点连线方向
	int contours_parallel_angle_max,contours_vertical_angle; //两轮廓方向角度之差
	int contours_ratio_min, contours_ratio_max; // 两轮廓长宽比
	// 寻找圆心
	int LED2R_DIR_MAX; //装甲到圆心的方向夹角
	int LED2R_DIST_MAX; int LED2R_DIST_MIN; // 装甲到圆心的距离
	// 预测角度
	int pre_theta = 30; int is_clockwise; // 是否顺时针转

	bool ifShow, if_calc_depth = 0, if_analyze_motion = 0;
	cv::Point2f last_dir;
	cv::Point2f last_center;
	int cnt;
	MarkerParams(bool ifShow_,bool isred_,int _is_clockwise) {
		//nh_.getParam("/AlgoriParams/is_enemy_red", is_red);
		//if (is_red) {
		//	nh_.getParam("/AlgoriParams/ch1_min_r", ch1_min);
		//	nh_.getParam("/AlgoriParams/ch1_max_r", ch1_max);
		//	nh_.getParam("/AlgoriParams/ch3_min_r", ch3_min);
		//	nh_.getParam("/AlgoriParams/ch3_max_r", ch3_max);
		//}
		//else {
		//	nh_.getParam("/AlgoriParams/ch1_min_b", ch1_min);
		//	nh_.getParam("/AlgoriParams/ch1_max_b", ch1_max);
		//	nh_.getParam("/AlgoriParams/ch3_min_b", ch3_min);
		//	nh_.getParam("/AlgoriParams/ch3_max_b", ch3_max);
		//}
		//nh_.getParam("/AlgoriParams/ch2_min", ch2_min);
		//nh_.getParam("/AlgoriParams/ch2_max", ch2_max);
		//nh_.getParam("/dbg_img_path", dbg_path);
		ifShow = ifShow_;
		is_red = isred_;
		is_clockwise = _is_clockwise;
		last_dir = cv::Point2f(0, 0);
		last_center= cv::Point2f(0, 0);
		cnt = 0;
		/*nh_.getParam("/if_calc_depth", if_calc_depth);
		nh_.getParam("/if_analyze_motion", if_analyze_motion);
		nh_.getParam("/MarkerParams/contours_lengtch1_min", contours_lengtch1_min);
		nh_.getParam("/MarkerParams/contours_lengtch1_max", contours_lengtch1_max);
		nh_.getParam("/MarkerParams/LED_ratio_min", LED_ratio_min);
		nh_.getParam("/MarkerParams/LED_ratio_max", LED_ratio_max);
		nh_.getParam("/MarkerParams/LED_widtch1_min", LED_widtch1_min);
		nh_.getParam("/MarkerParams/LED_widtch1_max", LED_widtch1_max);
		nh_.getParam("/MarkerParams/marker_parallel_angle", marker_parallel_angle);
		nh_.getParam("/MarkerParams/marker_vertical_angle", marker_vertical_angle);
		nh_.getParam("/MarkerParams/marker_direction_angle", marker_direction_angle);
		nh_.getParam("/MarkerParams/marker_ratio_min", marker_ratio_min);
		nh_.getParam("/MarkerParams/marker_ratio_max", marker_ratio_max);
		nh_.getParam("/MarkerParams/marker_size_min", marker_size_min);
		nh_.getParam("/MarkerParams/marker_size_max", marker_size_max);*/
		if (is_red) {
			ch1_min = 0; ch1_max = 33; ch2_min = 77; ch2_max = 255; ch3_min = 220; ch3_max = 255;
			contour_direction_angle_min = 0; contour_direction_angle_max = 180;
			// 第一个轮廓
			contour_lengtch1_min = 5; contour_lengtch1_max = 15;
			contour_widtch1_min = 1; contour_widtch1_max = 10;
			contour_ratio1_min = 1; contour_ratio1_max = 3;
			//第二个轮廓
			contour_lengtch2_min = 15; contour_lengtch2_max = 60;
			contour_widtch2_min = 10; contour_widtch2_max = 40;
			contour_ratio2_min = 2; contour_ratio2_max = 25;
			// 轮廓之间
			contours_dist_min = 1;  contours_dist_max = 16;
			contours_c2c_angle_max = 10;
			contours_parallel_angle_max = 30; contours_vertical_angle = 0;
			contours_ratio_min = 1; contours_ratio_max = 20;
			//寻找圆心
			LED2R_DIST_MIN = 33; LED2R_DIST_MAX = 61;
			LED2R_DIR_MAX = 15;
			// 预测角度
			pre_theta = 30; 
		}
		else {
			ch1_min = 80; ch1_max = 124; ch2_min = 54; ch2_max = 255; ch3_min = 220; ch3_max = 255;
			contour_direction_angle_min = 0; contour_direction_angle_max = 180;
			// 第一个轮廓
			contour_lengtch1_min = 5; contour_lengtch1_max = 15;
			contour_widtch1_min = 1; contour_widtch1_max = 10;
			contour_ratio1_min = 1; contour_ratio1_max = 3;
			//第二个轮廓
			contour_lengtch2_min = 15; contour_lengtch2_max = 60;
			contour_widtch2_min = 10; contour_widtch2_max = 40;
			contour_ratio2_min = 2; contour_ratio2_max = 25;
			// 轮廓之间
			contours_dist_min = 1;  contours_dist_max = 16;
			contours_c2c_angle_max = 10;
			contours_parallel_angle_max = 30; contours_vertical_angle = 0;
			contours_ratio_min = 1; contours_ratio_max = 20;
			//寻找圆心
			LED2R_DIST_MIN = 33; LED2R_DIST_MAX = 61;
			LED2R_DIR_MAX = 15;
			// 预测角度
			pre_theta = 30;
		}

	}
}; 
#endif