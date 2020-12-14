#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MarkerSensor.h"
using namespace std;
using namespace cv;

MarkSensor *markSensor = NULL;
bool is_red = 1, ifshow = 0;
int is_clockwise = -1;//顺时针+1，逆时针-1
int cam_idx = 1;
bool is_windMill_mode = false;

void paraAdjust(MarkSensor* mk){
	//MarkerParams mp = mk->mp;
	cvNamedWindow("debug", 0);
	cvCreateTrackbar("ch1_min", "debug", &mk->mp.ch1_min, 255);
	cvCreateTrackbar("ch1_max", "debug", &mk->mp.ch1_max, 255);
	cvCreateTrackbar("ch2_min", "debug", &mk->mp.ch2_min, 255);
	cvCreateTrackbar("ch2_max", "debug", &mk->mp.ch2_max, 255);
	cvCreateTrackbar("ch3_min", "debug", &mk->mp.ch3_min, 255);
	cvCreateTrackbar("ch3_max", "debug", &mk->mp.ch3_max, 255);
	cvCreateTrackbar("len1_min", "debug", &mk->mp.contour_lengtch1_min, 180);
	cvCreateTrackbar("len1_max", "debug", &mk->mp.contour_lengtch1_max, 180);
	//cvCreateTrackbar("dir_angle_min", "debug", &mk->mp.contour_direction_angle_min, 180);
	//cvCreateTrackbar("dir_angle_max", "debug", &mk->mp.contour_direction_angle_max, 180);
	cvCreateTrackbar("wid1_min", "debug", &mk->mp.contour_widtch1_min, 320);
	cvCreateTrackbar("wid1_max", "debug", &mk->mp.contour_widtch1_max, 320);
	cvCreateTrackbar("ratio1_min", "debug", &mk->mp.contour_ratio1_min, 30);
	cvCreateTrackbar("ratio1_max", "debug", &mk->mp.contour_ratio1_max, 30);

	cvCreateTrackbar("len2_min", "debug", &mk->mp.contour_lengtch2_min, 180);
	cvCreateTrackbar("len2_max", "debug", &mk->mp.contour_lengtch2_max, 180);
	cvCreateTrackbar("wid2_min", "debug", &mk->mp.contour_widtch2_min, 320);
	cvCreateTrackbar("wid2_max", "debug", &mk->mp.contour_widtch2_max, 320);
	cvCreateTrackbar("ratio2_min", "debug", &mk->mp.contour_ratio2_min, 30);
	cvCreateTrackbar("ratio2_max", "debug", &mk->mp.contour_ratio2_max, 30);

	cvCreateTrackbar("dist_min", "debug", &mk->mp.contours_dist_min, 320);
	cvCreateTrackbar("dist_max", "debug", &mk->mp.contours_dist_max, 320);
	cvCreateTrackbar("para_angle_max", "debug", &mk->mp.contours_parallel_angle_max, 180);
	cvCreateTrackbar("c2c_angle", "debug", &mk->mp.contours_c2c_angle_max, 180);
	cvCreateTrackbar("ratio_min", "debug", &mk->mp.contours_ratio_min, 30);
	cvCreateTrackbar("ratio_max", "debug", &mk->mp.contours_ratio_max, 30);
	//寻找圆心
	cvCreateTrackbar("L2R_dist_min", "debug", &mk->mp.LED2R_DIST_MIN, 320);
	cvCreateTrackbar("L2R_dist_max", "debug", &mk->mp.LED2R_DIST_MAX, 320);
	cvCreateTrackbar("L2R_dir_max", "debug", &mk->mp.LED2R_DIR_MAX, 180);
	cvCreateTrackbar("pre_theta", "debug", &mk->mp.pre_theta, 180);
}
int main(int argc, char** argv) {
	//nh_.getParam("/ifshow", ifshow);
	//nh_.getParam("/cam_idx", cam_idx);
	CamParams cp(cam_idx, !is_windMill_mode);
	MarkerParams mp(ifshow,is_red, is_clockwise);
	markSensor = new MarkSensor(mp,cp);
	paraAdjust(markSensor);
	//-----------------------------------------
	//VideoCapture cap(1);
	VideoCapture cap; 
	if (is_red) {
		cap.open("red1.mp4");
	}
	else cap.open("blue1.mp4");
	if (!cap.isOpened()) {
		cout << "Failed to open camera." << endl;
		return -1;
	}
	int framesNum = cap.get(CV_CAP_PROP_FRAME_COUNT); int framesCnt = 0;
	//帧率计算
	int timeStamp[2];
	timeStamp[1] = getTickCount();
	for (;;) {
		timeStamp[0] = getTickCount();
		float timeFly = (timeStamp[0] - timeStamp[1]) / getTickFrequency();
		float fps1 = 1 / timeFly;
		timeStamp[1] = getTickCount();
		cout << "---FPS:" << fps1 << endl;
		//开始读取
		if (framesCnt++ == framesNum - 1) {
			cap.set(CV_CAP_PROP_POS_FRAMES, 0);
			framesCnt = 0;
		}
		Mat frame; cap >> frame;
		Mat Img;
		//frame = imread("red4.png");
		if (frame.empty()) continue;
		frame.copyTo(Img);
		markSensor->img_show = frame;
		//markSensor->detectTrackFrame(Img);
		markSensor->DetectMarker(Img, markSensor->marker);
		//结束
		namedWindow("frame", 0);
		imshow("frame", markSensor->img_show);
		
		if (waitKey(30) >= 0)
			continue;
	}
	return 0;

}
int frameProcess() {
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		cout << "Failed to open camera." << endl;
		return -1;
	}
	//帧率计算
	int timeStamp[2];
	timeStamp[1] = getTickCount();
	for (;;) {
		timeStamp[0] = getTickCount();
		float timeFly = (timeStamp[0] - timeStamp[1]) / getTickFrequency();
		float fps1 = 1 / timeFly;
		timeStamp[1] = getTickCount();
		cout << "---FPS:" << fps1 << endl;
		//开始读取
		Mat frame; cap >> frame;
		Mat dstImage, grayImage, binaryImage;
		frame.copyTo(dstImage);
		cvtColor(dstImage, grayImage, COLOR_BGR2GRAY);
		int blockSize = 25; int constValue = 10;
		adaptiveThreshold(grayImage, binaryImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
		vector<vector<Point> > contours; vector<Vec4i> hierarchy;
		findContours(binaryImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		vector< vector<Point> >::iterator It;
		for (It = contours.begin(); It < contours.end(); It++) {
			if (It->size() < 5) continue;
			Rect rect = boundingRect(*It); //画出可包围数字的最小矩

		}

		//结束
		imshow("frame", frame);
		if (waitKey(30) >= 0)
			continue;
	}
	return 0;
}