#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"  
#include "opencv2/objdetect/objdetect.hpp"
using namespace cv;
#include <iostream>
#include <algorithm>
#include <vector> 
#include <string>
#include <list>
#include <fstream> 
#include <iomanip>
#include <math.h>
#include <io.h>
#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <map>
using namespace std;


#define match_t 0.6

struct assist_registration {
	int distance_to_left=9999;
	Mat match_line_image;
	Mat homography;
	vector<vector<double>> points;
	bool flag;
};
struct xy_feature {
	int x;
	int y;
	double dx;
	double dy;
};
struct assist_information {
	// 基本信息
	Mat base_image;
	Mat wrap_image;
	int length;
	int ruler_number=1;
	// 用于配准
	vector<KeyPoint> keypoints;
	Mat descriptors;
	// roi 
	Mat assist_image;
	vector<double> roi;
	vector<double> sub_roi;
	int roi_order = 1;
	// 摄像头抖动
	bool correct_flag = false;
	double correct_score = -1;
	vector<vector<double>> temp_correct_point;
	vector<vector<double>> correct_point;
	Point2f base_point{ 0, 0 };
	//float correct_line;
	// 校正用点
	bool correct2poly = true;
	vector<vector<double>> point;
	Vec4f line_para;
	vector<int> rect;
	Mat r;
	Mat r_inv;
	double rms_1, rms_2;
	Mat mask;
	//ref
	int ref_index;
	Mat ref_image;
	// left_right_water
	bool left_right_no_water;
	// 分割水位线
	Mat segment_result;
	Mat left_image;
	Mat right_image;
	Mat expand_wrap_image;
	vector<float> scores1;
	vector<float> scores2;
	vector<Point2d> parrallel_lines;
	vector<Point2d> parrallel_left;
	vector<Point2d> parrallel_right;
	vector<double> water_lines;
	double water_line = -1;
	double water_number = -1;
	// 
	vector<vector<float>> scores;
};

// 读入辅助信息
vector<string> getFiles(string folder, string firstname, string lastname);
bool input_assist(Mat im,map<string, string> main_ini, vector<assist_information> &assist_files, bool assist_flag = false);
bool get_number(string line_string, vector<double> &temp);
bool input_assist_image(string file_name,assist_information &assist_file);
bool get_roi(vector<assist_information> &assist_files, map<string, string> main_ini);
// 处理主函数
void compute_water_area(Mat im, vector<assist_information> &assist_files,string ref_name);
void opt_assist_files(vector<assist_information> &assist_files);
// 判断是白天还是黑夜
bool isgrayscale(Mat im);
// 判断是否全是水
bool isblank(Mat im, assist_information &assist_file);
// 判断是否已经是底部了，白天；判断左右是否有水，晚上；
bool notall(Mat im, assist_information &assist_file);
// 夜晚是否过亮 并优化
bool isTooHighLightInNight(Mat im,int &water_line,int &gray_value);
// 对原始影像进行配准

// 几何校正原始影像
Mat correct_image(Mat im, assist_information &assist_file);
void map_coord(assist_information &assist_file,Mat &map_x, Mat &map_y,int base_x = 0, int base_y = 0);
Mat GeoCorrect2Poly(assist_information assist_file,bool flag);
Mat compute_point(Mat point, Mat r);
double compute_rms(Mat base_point, Mat wrap_point, Mat r);

// 夜晚水位线
float get_water_line_night(Mat im,assist_information &assist_file);
float get_water_line_night_local(Mat im,assist_information &assist_file);
bool left_right_water(Mat gc_im,int length);
int get_water_line_seg(Mat im, int length, int add_rows = 100, float scale = 0.2);
// 结果保存
void save_file(Mat im, vector<assist_information> assist_files,map<string,string> main_ini);
void save_maskfile(vector<assist_information> assist_files, map<string, string> main_ini);
// 功能函数
vector<vector<double>> part_row_point(vector<vector<double>> point,int r1,int r2);
vector<vector<double>> part_col_point(vector<vector<double>> point,int c1,int c2);
vector<vector<double>> swap_point(vector<vector<double>> &data);
Mat vector2Mat(vector<vector<double>> data);
vector<vector<double>> Mat2vector(Mat data);
// 
void get_parrallel_lines(assist_information &assist_file);
vector<double>  interpolate_point(vector<double>&point1, vector<double>& point2,int y);
Point2d change_point_by_line(Point2d& first, Point2d& last, Point2d point);

Mat get_ref_index(assist_information& assist_file, Mat mask);
double linetonumber(assist_information& assist_file);
class Feature
{
public:
	/// X location.
	int x;
	/// Y location.
	int y;
	///X normalized first derivative
	float dx;
	///Y normalized first derivative
	float dy;
	// 用于计算相似时
	bool flag = false;

public:
	Feature();
	Feature(int x, int y, float dx, float dy, bool flag);
	~Feature(void);
	// Clones the feature.
	Feature& operator= (Feature& feature);

};
//// 模板制作
vector<Feature> make_template(Mat image, Rect &base_rect);
//// 模板匹配
void find_object(Mat search_image, vector<Mat> merge_im,
	Mat template_image, Rect base_rect, vector<Feature> contours,
	Point & result_point, float& score);
//// 功能函数
Mat draw_contours(Mat im, vector<Feature> contors, Point xy = Point(0, 0), int width = 1, Scalar color = Scalar{ 0,255,0 });
// 得到范围
Rect GetBoundingRectangle(vector<Feature> contours);
// 图像阈值 自适应
int TresholdOtsu(Mat im);
// 得到合适的阈值
void  GetImageThreshold(Mat image, int &bestLowThresh, int &bestHighThresh);
// 计算待搜索图像梯度
vector<Mat> cal_search_feature(Mat image, int minGradientMagnitude = 35);

