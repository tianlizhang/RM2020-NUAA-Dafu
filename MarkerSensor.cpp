#include "MarkerSensor.h"
using namespace cv;
using namespace std;

/*
theta = asin(x) 定义域x[-1,1],值域弧度,[-pi/2,pi/2]
theta = acos(x) 定义域x[-1,1],值域弧度,[0,pi]
theta = atan(x) 定义域x[-inf,inf],值域弧度,[-pi/2,pi/2]
theta = atan2(x) 定义域x[-inf,inf],值域弧度,(-pi,pi],与x轴方位角 
*/

int MarkSensor::getPitchYawDegree(float horizontal_dist, float horizontal_diff, float height_diff, float BulletVelocit, float &PitchDegree,float &YawDegree) {
	float L = horizontal_dist; float h = height_diff; float v = BulletVelocit; float g = 9.8;
	float b = -g - 4 * L*L*h / (v*v); //中间变量
	float delta = 4 * L*L + g*g - b*b;
	if (delta < 0) return 0;
	float x = (-2*L*b+g*sqrt(delta)) / (4 * L*L + g*g);// 设x=sin(2theta),解一元二次可得
	if (x < 0 || x>1) return 0;
	PitchDegree = asin(x) / 2;
	PitchDegree *= RAD2ANG; // 弧度转角度
	YawDegree = atan(horizontal_diff/L);
	return 1;
}
int MarkSensor::focus2Lines(Point2f dir1, Point2f pt1, Point2f dir2, Point2f pt2, Point2f &res) {
	float a1 = dir1.y; float b1 = -dir1.x; float c1 = a1*pt1.x + b1*pt1.y; // a1*x+b1*y=c1
	float a2 = dir2.y; float b2 = -dir2.x; float c2 = a2*pt2.x + b2*pt2.y; // a2*x+b2*y=c2
	float e = a1*b2 - a2*b1;
	if (e == 0) {
		res.x = 0;
		res.y = 0;
		return 0;
	}
	else {
		res.x = (c1*b2-c2*b1) / e;
		res.y = (a1*c2-a2*c1) / e;
		return 1;
	}
}
float MarkSensor::ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir) {
	float max_range = -999999;
	float min_range = 999999;
	for (auto & pt : contour) {
		float x = pt.x*dir.x + pt.y*dir.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
		if (x < min_range) min_range = x;
		if (x > max_range) max_range = x;
	}
	return (max_range - min_range);
}
int MarkSensor::PCAMarker(vector<cv::Point> &pts, Point2f &dir1,Point2f &dir2,Point2f &center) {
	int sz = static_cast<int>(pts.size());
	cv::Mat data_pts(sz, 2, CV_64FC1);
	for (int i = 0; i < data_pts.rows; ++i) {
		data_pts.at<double>(i,0) = pts[i].x;
		data_pts.at<double>(i,1) = pts[i].y;
	}
	cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW); // 特征向量代表主成分方向，已归一化
	center.x = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
	center.y = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
	dir1.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 0)); // 与x轴下方的夹角[0,pi]的余弦
	dir1.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 1)); // 与x轴下方的夹角[0,pi]的正弦
	dir2.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 0));
	dir2.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 1));
	int is_dir1_down = (dir1.dot(Point2f(0, 1)))>0 ? 1 : -1; //规定方向朝下，即与x轴下方的夹角[0,pi]
	int is_dir2_down = (dir2.dot(Point2f(0, 1)))>0 ? 1 : -1; //规定方向朝下，即与x轴下方的夹角[0,pi]
	dir1 = is_dir1_down*dir1;
	dir2 = is_dir2_down*dir2;
	return 0;
}
int MarkSensor::chooseMarker(vector<Marker> markers) {
	int sz = markers.size(); // 思想：选择dir2投影方向上最小的marker，即最靠近圆心的marker
	if (sz < 2) return 0;
	Point2f dir2 = markers[0].LEDs[1].center - markers[0].LEDs[0].center; //圆心为起点，led为终点的方向
	dir2 /= norm(dir2); // 归一化
	float min_range = 999999;
	int min_id = 0;
	for (int i = 0; i < sz; i++) {
		Point2f pt = markers[i].LEDs[1].center;
		float x = pt.x*dir2.x + pt.y*dir2.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
		if (x < min_range) {
			min_range = x;
			min_id = i;
		}
	}
	return min_id;
}
int MarkSensor::getCircleCenter(Point2f &circle_center, Marker &marker, vector <RotRect> boxs) {
	if (boxs.empty()) { //boxs: 疑似圆心的盒子
		circle_center = Point2f(0, 0);
		return 0;
	}
	Point2f dir_LED2R = marker.LEDs[0].center - marker.LEDs[1].center; // 装甲到圆心R的方向向量
	vector <RotRect> res_box;
	for (auto &box:boxs) {
		Point2f dir = box.center - marker.LEDs[1].center;
		float dist = norm(dir); // 1、装甲到圆心的距离
		if (dist < mp.LED2R_DIST_MIN || dist>mp.LED2R_DIST_MAX) continue; 
		dir /= dist; // 2、装甲到圆心的夹角方向
		if (acos(fabs(dir.dot(marker.LEDs[1].dir2))) > mp.LED2R_DIR_MAX / RAD2ANG) continue; 
		res_box.push_back(box);
		//box.Draw(img_show);
	}
	if (res_box.empty()) { // 根据比例对圆心进行估计
		float LED2R_RATIO = 700 / 260; // 装甲到圆心距离/装甲宽度
		circle_center = marker.LEDs[1].center + marker.LEDs[1].width*LED2R_RATIO*dir_LED2R/norm(dir_LED2R);
		return 1;
	}
	float max_range = 999999; int max_id = 0; // 选择离装甲最远的盒子作为圆心
	for (int i = 0; i < res_box.size(); ++i) {
		RotRect box = res_box[i];
		float x = box.center.x*dir_LED2R.x + box.center.y*dir_LED2R.y;// project pix on dir
		if (x > max_range) {
			max_range = x;
			max_id = i;
		}
	}
	circle_center = res_box[max_id].center;
	//res_box[max_id].Draw(img_show);
	return 1;
}
Point2f MarkSensor::predictObject(int preTheta,Point2f &circle_center, Marker &marker) {
	float LED_RATIO = 86 / 280;
	Point2f dir_R2LED = marker.LEDs[1].center - circle_center; //圆心到装甲的向量
	float radius = norm(dir_R2LED);
	circle(img_show, circle_center, radius, Scalar(255, 255, 255));
	Point2f armor_center;
	armor_center.x = dir_R2LED.x*cos(preTheta / RAD2ANG) - dir_R2LED.y*sin(preTheta / RAD2ANG) + circle_center.x;
	armor_center.y = dir_R2LED.x*sin(preTheta / RAD2ANG) + dir_R2LED.y*cos(preTheta / RAD2ANG) + circle_center.y;
	circle(img_show, armor_center, 2, Scalar(0, 255, 0));
	return armor_center;
}
int MarkSensor::GetMarker(cv::Mat &roi_mask, Marker &res_marker) {
	vector<vector<Point>> countours; vector<Vec4i> hierarchy;
	findContours(roi_mask, countours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int contoursid = 0;
	vector <RotRect> boxs[2];// 最小包围轮廓旋转矩形
	if (countours.empty()) {
		cout << "no contours!" << endl; return -1;
	}
	for (auto &contour : countours) {
		//1、轮廓长度
		RotRect box;
		int cont_sz = static_cast<int>(contour.size());
		if (cont_sz < mp.contour_lengtch1_min || cont_sz > mp.contour_lengtch1_max) goto CONTOUR2;
		//2、轮廓方向：PCA
		PCAMarker(contour, box.dir1, box.dir2, box.center); 
		//if (acos(box.dir.x) > (mp.contour_direction_angle_max / RAD2ANG) ||
		//	acos(box.dir.x) < (mp.contour_direction_angle_min / RAD2ANG)) continue; //theta=acos x 值域弧度[0,pi],定义域x[-1,1]
		//3、最小包围矩形长宽
		box.width = ComputeLengthAlongDir(contour, box.dir1);
		box.height = ComputeLengthAlongDir(contour, box.dir2);
		if (box.width < mp.contour_widtch1_min || box.width > mp.contour_widtch1_max) goto CONTOUR2;
		//4、最小包围矩形长宽比
		float ratio = box.width / box.height;   //>1
		if (ratio < mp.contour_ratio1_min || ratio > mp.contour_ratio1_max) goto CONTOUR2;
		boxs[0].push_back(box);
		if (mp.ifShow) {
			//cout << "contour_widtch1_min:" << mp.contour_widtch1_min << endl;
			//drawContours(img_show, countours, contoursid, Scalar(255), 1, 8, hierarchy);
			//contoursid++;
			//box.Draw(img_show);
		}
		continue;
CONTOUR2:
		//1、轮廓长度
		if (cont_sz < mp.contour_lengtch2_min || cont_sz > mp.contour_lengtch2_max) continue;
		//2、轮廓方向：PCA
		RotRect box2; 
		PCAMarker(contour, box2.dir1, box2.dir2, box2.center);
		//if (acos(box2.dir.x) > (mp.contour_direction_angle_max / RAD2ANG) ||
		//	acos(box2.dir.x) < (mp.contour_direction_angle_min / RAD2ANG)) continue; //theta=acos x 值域弧度[0,pi],定义域x[-1,1]
		//3、最小包围矩形长宽
		box2.width = ComputeLengthAlongDir(contour, box2.dir1);
		box2.height = ComputeLengthAlongDir(contour, box2.dir2);
		if (box2.width < mp.contour_widtch2_min || box2.width > mp.contour_widtch2_max) continue;
		//4、最小包围矩形长宽比
		float ratio2 = box2.width / box2.height;   //>1
		if (ratio2 < mp.contour_ratio2_min || ratio2 > mp.contour_ratio2_max) continue;
		boxs[1].push_back(box2);
		if (mp.ifShow) {
			//cout << "contour_widtch1_min:" << mp.contour_widtch1_min << endl;
			//drawContours(img_show, countours, contoursid, Scalar(255), 1, 8, hierarchy);
			//contoursid++;
			//box2.Draw(img_show);
		}
	}
	if (boxs[0].size() < 1 || boxs[1].size()<1) {
		//printf("box num < 2 ! \n");
		return -1;
	}
	/*namedWindow("led_mask", 0);
	imshow("led_mask", led_mask);
	return 0;*/
	// 两轮廓之间关系
	vector<bool> matched1(boxs[0].size(), false); //是否匹配
	vector<bool> matched2(boxs[1].size(), false); //是否匹配
	vector<Marker> markers; // 存储合适的 Marker
	for (size_t i = 0; i < boxs[0].size(); ++i) {
		if (matched1[i]) continue; // 防止重复配对
		for (size_t j = 0; j < boxs[1].size(); ++j) {
			if (matched2[j]) continue; // 防止重复配对
			// 1、之间距离
			float distance = norm(boxs[0][i].center - boxs[1][j].center); //norm 求两点之间距离
			if (distance > mp.contours_dist_max || distance < mp.contours_dist_min) continue;
			// 2、中心点连线方向
			Point2f c2c_dir = (boxs[0][i].center - boxs[1][j].center)/ distance; // 中心点连线归一化向量
			float d_angle = abs(acos(fabs(c2c_dir.dot(boxs[1][j].dir1)))-CV_PI/2); // 与90度的差距
			if (d_angle > (mp.contours_c2c_angle_max / RAD2ANG)) continue;
			// 3、方向角度之差
			if (acos(fabs(boxs[0][i].dir1.dot(boxs[1][j].dir1))) > (mp.contours_parallel_angle_max/RAD2ANG)) continue;
			// 4、长宽比
			float marker_height = (boxs[0][i].width + boxs[1][j].width)*0.5f;
			float ratio = distance / marker_height;
			if (ratio > mp.contours_ratio_max || ratio < mp.contours_ratio_min) continue;
			// 存储
			matched1[i] = matched2[j] = true;
			Marker tmp_marker;
			if (boxs[0][i].width > boxs[1][j].width) {
				tmp_marker.LEDs[0] = boxs[1][j];//0是小的那个轮廓
				tmp_marker.LEDs[1] = boxs[0][i];
			}
			else {
				tmp_marker.LEDs[0] = boxs[0][i];
				tmp_marker.LEDs[1] = boxs[1][j];
			}
			markers.push_back(tmp_marker);
			if (mp.ifShow) {
				/*if(boxs[0][i].width>boxs[1][j].width) boxs[0][i].Draw(img_show);
				else boxs[1][j].Draw(img_show);*/
				//tmp_marker.Draw(img_show);
			}
		}
	}
	if (mp.ifShow) {
		namedWindow("led_mask", 0);
		imshow("led_mask", led_mask);
	}
	if (markers.empty()) return -1;
	int res_idx = chooseMarker(markers); // 选择哪个Marker
	res_marker = markers[res_idx]; res_marker.Draw(img_show);
	res_marker.ComputeKeyPoints();
	res_marker.ComputeBBox(); // 计算新的bbox
	//寻找圆心
	Point2f circle_center;
	bool is_getcc = getCircleCenter(circle_center, res_marker, boxs[0]);
	circle(img_show, circle_center, 3, Scalar(255, 255, 255));
	// 预测
	if (is_getcc) {
		Point2f obj_point = predictObject(mp.is_clockwise*mp.pre_theta, circle_center, res_marker);
		circle(img_show, obj_point, 3, Scalar(255, 255, 255));
	}
	return is_getcc-1;
}
int MarkSensor::DetectMarker(const Mat &img, Marker &res_marker) {
	img.copyTo(img_bgr);
	cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
	inRange(img_hsv,Scalar(mp.ch1_min, mp.ch2_min, mp.ch3_min), Scalar(mp.ch1_max, mp.ch2_max, mp.ch3_max), led_mask);
	//Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	//morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);
	if (mp.ifShow) {
		//cout << "mp.ch1_min:" << mp.ch1_min << endl;
		//imshow("led_mask", led_mask);
	}
	bool is_detected = GetMarker(led_mask, res_marker);
	if (!is_detected) {
		// dbg_save(img,ap.dbg_path,status);
	}
	
	return is_detected;
}
int MarkSensor::TrackMarker(const Mat &img, Marker &res_marker) {
	Rect &box = res_marker.bbox; //bbox为ROI
	float left = box.x - status*box.width;
	float right = box.x + box.width * (status + 1);
	float top = box.y - status*box.height;
	float bot = box.y + box.height * (status + 1);
	left = left < 0 ? 0 : left;
	right = right >= img.cols ? img.cols : right;
	top = top < 0 ? 0 : top;
	bot = bot >= img.rows ? img.rows : bot;
	Rect ROI(left, top, (right - left), (bot - top));
	Mat imgRoi = img(ROI).clone();
	if (imgRoi.empty()) {
		printf("no marker for tracking!!");
		status = STATUS_DETECTING;
		marker = Marker();
		return -1;
	}
	//-------------------------
	Mat ROI_led_mask;
	imgRoi.copyTo(img_bgr);
	cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
	inRange(img_hsv, Scalar(mp.ch1_min, mp.ch2_min, mp.ch3_min),Scalar(mp.ch1_max, mp.ch2_max, mp.ch3_max), ROI_led_mask);
	if (GetMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) { // STATUS_SUCCESS=0;
		printf("Get no marker!\n"); return -1;
	}
	//------------------------
	res_marker.LEDs[0].center.x += ROI.x; //变换回去
	res_marker.LEDs[0].center.y += ROI.y;
	res_marker.LEDs[1].center.x += ROI.x;
	res_marker.LEDs[1].center.y += ROI.y;
	res_marker.ComputeKeyPoints();
	res_marker.ComputeBBox(); // 计算新的bbox
	if (mp.ifShow) {
		rectangle(img_show, res_marker.bbox, Scalar(0, 255, 0), 2);
	}
	return 0;
}

int MarkSensor::detectTrackFrame(const Mat &img) {
	if (status == STATUS_DETECTING) {
		if (DetectMarker(img, marker) == STATUS_SUCCESS) {
			status = STATUS_TRACKING;
		}
		else {
			printf("Detect No target!\n");
			return -1;
		}
	}
	else if (status == STATUS_TRACKING) {
		if (TrackMarker(img, marker) == STATUS_SUCCESS) {
			printf("Track Success!\n");
		}
		else {
			status = STATUS_TRACKLOST0;
			printf("Track No target!\n");
			track_fail_cnt[0] = 0;
			marker=Marker();
			return -1;
		}
	}
	//else if (status == STATUS_TRACKLOST0) {
	//	if (TrackMarker(img, marker) == STATUS_SUCCESS) {
	//		printf("Track 0 Success!\n");
	//		status = STATUS_TRACKING;
	//	}
	//	else {
	//		printf("Track 0 No target!\n");
	//		track_fail_cnt[0]++;
	//		if (track_fail_cnt[0]>10) {
	//			status = STATUS_TRACKLOST1;
	//			printf("enlarge ROI!");
	//			track_fail_cnt[0] = 0;
	//			track_fail_cnt[1] = 0;
	//		}
	//		//marker=Marker();
	//		return -1;
	//	}
	//}
	//else if (status == STATUS_TRACKLOST1) {
	//	if (TrackMarker(img, marker) == STATUS_SUCCESS) {
	//		printf("Track 0 Success!\n");
	//		status = STATUS_TRACKING;
	//	}
	//	else {
	//		printf("Track 1 No target!\n");
	//		track_fail_cnt[1]++;
	//		if (track_fail_cnt[1]>10) {
	//			status = STATUS_TRACKLOST2;
	//			printf("ROI enlarge again!");
	//			track_fail_cnt[1] = 10;
	//			track_fail_cnt[2] = 0;
	//		}
	//		//marker=Marker();
	//		return -1;
	//	}
	//}
	//else if (status == STATUS_TRACKLOST2) {
	//	if (TrackMarker(img, marker) == STATUS_SUCCESS) {
	//		printf("Track 0 Success!\n");
	//		status = STATUS_TRACKING;
	//	}
	//	else {
	//		printf("Track 0 No target!\n");
	//		track_fail_cnt[2]++;
	//		if (track_fail_cnt[2]>20) {
	//			status = STATUS_DETECTING;
	//			printf("failed to find marker in ROI");
	//			track_fail_cnt[2] = 0;
	//			marker = Marker();
	//		}
	//		return -1;
	//	}
	//}
}
