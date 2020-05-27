//
#define torch_1_4
#define mask_flag true
#pragma once
#include "xy_torch.h"
#include <iostream>
#include <algorithm>
#include <vector> 
#include <string>
#include <list>
#include <fstream>
#include <map>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/core/core.hpp"

using namespace cv;

struct assist_information {
	// 基本信息
	Mat base_image;
	Mat wrap_image;
	int length;
	int ruler_number = 1;
	// 用于配准
	std::vector<KeyPoint> keypoints;
	Mat descriptors;
	// roi 
	Mat assist_image;
	std::vector<double> roi;
	std::vector<double> sub_roi;
	int roi_order = 1;
	// 摄像头抖动
	bool correct_flag = false;
	double correct_score = -1;
	std::vector<std::vector<double>> temp_correct_point;
	std::vector<std::vector<double>> correct_point;
	Point2d base_point{ 0, 0 };
	Point2d left_point{ 0, 0 };
	Point2d right_point{ 0, 0 };
	//float correct_line;
	// 校正用点
	bool correct2poly = true;
	std::vector<std::vector<double>> point;
	Vec4f line_para;
	std::vector<int> rect;
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
	std::vector<float> scores1;
	std::vector<float> scores2;
	std::vector<Point2d> parrallel_lines;
	std::vector<Point2d> parrallel_left;
	std::vector<Point2d> parrallel_right;
	std::vector<double> water_lines;
	double water_line = -1;
	double water_number = -1;
	// 
	std::vector<std::vector<float>> scores;
};

bool get_number(std::string line_string, std::vector<double>& temp)
{
	temp.clear();
	for (int i = 0; i < line_string.size(); ++i) {
		if ((line_string[i] >= 48 && line_string[i] <= 57) || line_string[i] == 45) {
			double temp_value = 0;
			int j = i;
			int n = -1;
			bool flag = true;
			for (; j < line_string.size(); ++j) {
				if (line_string[j] == 45) {
					flag = false;
					continue;
				}
				if (line_string[j] == 44 || line_string[j] == 59) {
					break;
				}
				if (line_string[j] >= 48 && line_string[j] <= 57) {
					temp_value = temp_value * 10 + line_string[j] - 48;
				}
				if (line_string[j] == 46)
					n = j;
			}
			temp_value = n == -1 ? temp_value : temp_value / pow(10, j - n - 1);
			if (!flag)
				temp_value = temp_value * -1;
			temp.push_back(temp_value);
			i = j;

		}
	}
	return true;
}
bool input_assist_image(std::string file_name, assist_information &assist_file)
{
	std::string temp = std::string(file_name.begin(), file_name.end() - 4) + "_" + std::to_string(assist_file.ref_index) + std::string(file_name.end() - 4, file_name.end());
	Mat temp_image = cv::imread(temp, cv::IMREAD_COLOR);
	if (!temp_image.data)
		return false;
	assist_file.assist_image = temp_image.clone();
	return true;
}
bool input_assist(Mat im, std::map<std::string, std::string> main_ini, std::vector<assist_information> & assist_files, bool assist_flag)
{
	std::fstream assist_file_name(main_ini["assist_txt"]);
	if (!assist_file_name)
		return false;
	// 用于边缘检测时排除干扰
	Mat temp_im = im.clone();
	//
	assist_files.clear();
	std::string temp_name;
	getline(assist_file_name, temp_name);
	std::vector<double> temp;
	get_number(temp_name, temp);
	int ruler_number = temp[0];
	std::vector<assist_information> temp_assist_files;
	while (!assist_file_name.eof())
	{
		temp.clear();
		assist_information temp_assist_file;
		temp_assist_file.ruler_number = ruler_number;
		temp_assist_file.correct_flag = false;
		// 读取头文件
		getline(assist_file_name, temp_name);
		get_number(temp_name, temp);
		if (temp.size() != 5) {
			if (temp.size() == 6) {
				temp_assist_file.roi_order = temp[3];
			}
			else
				break;
		}
		temp_assist_file.left_right_no_water = temp[temp.size() - 2];
		int n_water = temp[0];
		temp_assist_file.ref_index = temp[1];
		//input_assist_image(main_ini["assist_image"], temp_assist_file);
		// 读取模板图片
		std::string temp_name = std::string(main_ini["template"].begin(), main_ini["template"].end() - 4)
			+ "_" + std::to_string((int)temp[2]) + std::string(main_ini["template"].end() - 4, main_ini["template"].end());
		Mat template_image = cv::imread(temp_name, cv::IMREAD_GRAYSCALE);
		if (!template_image.data) {
			std::cout << "template path error!" << std::endl;
			return false;
		}
		temp_assist_file.base_image = template_image;
		temp_assist_file.length = temp[2];
		// roi
		std::vector<double> roi;
		getline(assist_file_name, temp_name);
		get_number(temp_name, roi);
		if (roi.size() != 4)
			break;
		temp_assist_file.roi = roi;
		// sub_roi
		std::vector<double> sub_roi;
		getline(assist_file_name, temp_name);
		get_number(temp_name, sub_roi);
		if (sub_roi.size() != 4)
			break;
		temp_assist_file.sub_roi = sub_roi;
		// point
		std::vector<std::vector<double>> temp_point;
		for (int n = 0; n < temp[temp.size() - 1]; ++n) {
			getline(assist_file_name, temp_name);
			std::vector<double> temp;
			get_number(temp_name, temp);
			temp_point.push_back(temp);
		}
		temp_assist_file.point = temp_point;
		temp_assist_file.water_number = 0;
		//
		temp_assist_files.push_back(temp_assist_file);
	}
	if (assist_flag) {
		assist_files = temp_assist_files;
		return true;
	}
	return false;
}


int main(int argc, char** argv)
{
	if (argc < 3) {
		std::cout << "please input image name\n";
		std::cout << "please input pt name\n";
		return 0;
	}
	std::string image_name(argv[1]);
	Mat image = cv::imread(image_name, cv::IMREAD_COLOR);
	if (!image.data)
	{
		std::cout << "no image data" << std::endl;
		return -1;
	}
	std::string model_name = std::string(argv[2]);
	std::ifstream fin(model_name);
	if (!fin)
		return 1;
	fin.close();
	// 取出无后缀的文件名及文件所在文件夹路径
	std::string base_name, base_path;
	for (int i = image_name.size(); i >= 0; --i) {
		if (image_name[i] == 47 || image_name[i] == 92) {
			base_name.append(image_name.begin() + i + 1, image_name.end() - 4);
			base_path.append(image_name.begin(), image_name.begin() + i + 1);
			break;
		}
	}
	if (base_path == "") {
		base_name.append(image_name.begin(), image_name.end() - 4);
	}
	// 一些应该存在的重要文件所在路径
	std::string assist_txt_name(base_path + "assist_" + base_name + ".txt");
	std::string mask_image_name(base_path + "mask_" + base_name + ".png");
	std::string template_image_name(base_path + "template.png");

	std::map<std::string, std::string> main_ini;
	main_ini.insert(std::map<std::string, std::string>::value_type("mask_image", mask_image_name));
	main_ini.insert(std::map<std::string, std::string>::value_type("assist_txt", assist_txt_name));
	main_ini.insert(std::map<std::string, std::string>::value_type("template", template_image_name));


	for (int i = 3; i < argc; ++i) {
		std::string temp(argv[i]);
		std::string temp1, temp2;
		for (int j = 0; j < temp.size(); ++j) {
			if (temp[j] == 61) {
				temp1.append(temp.begin(), temp.begin() + j);
				temp2.append(temp.begin() + j + 1, temp.end());
			}
		}
		if (temp2 != "") {
			main_ini[temp1] = temp2;
		}
	}
	xy_torch xy;
	bool modelflag;
	modelflag = xy.load_model(model_name);
	if (!modelflag) {
		std::cout << "load model error\n";
	}
	//根据assist程序来进行处理
	std::vector<assist_information> assist_files;
	bool flag = input_assist(image, main_ini, assist_files, mask_flag);
	Mat mask = Mat::zeros(image.size(), CV_8UC1);

	if (!flag) {
		mask = xy.process_image(image);
		mask.setTo(0, mask < 100);
		mask.setTo(255, mask > 100);
	}
	else {
		for (auto assist_file : assist_files) {
			int min_x = assist_file.roi[0] >= 0 ? assist_file.roi[0] : 0;
			int min_y = assist_file.roi[1] >= 0 ? assist_file.roi[1] : 0;
			int max_x = assist_file.roi[0] + assist_file.roi[2] <= image.cols ? assist_file.roi[0] + assist_file.roi[2] : image.cols;
			int max_y = assist_file.roi[1] + assist_file.roi[3] <= image.rows ? assist_file.roi[1] + assist_file.roi[3] : image.rows;
			Mat temp_im = image(cv::Range(min_y, max_y), cv::Range(min_x, max_x)).clone();
			Mat result = xy.process_image(temp_im);
			for (int i = min_y; i < max_y; ++i)
				for (int j = min_x; j < max_x; ++j) {
					if (result.at<uchar>(i - min_y, j - min_x) > 100)
						mask.at<uchar>(i, j) = 255;
				}

		}
	}



	cv::imwrite(main_ini["mask_image"], mask);


	return 1;
}
