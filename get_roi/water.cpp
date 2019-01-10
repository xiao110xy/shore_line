
#include "water.h"
#define no_water_flag false
#define night_way false
#define MAXFEANUM 1500//最大特征数目

vector<string> getFiles(string folder, string firstname, string lastname)
{
	vector<string> files;
	//文件句柄  
	long long hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;   //大家可以去查看一下_finddata结构组成                            
								   //以及_findfirst和_findnext的用法，了解后妈妈就再也不用担心我以后不会编了  
	string p(folder);
	if (folder == "")
		p.append(".");
	p.append("\\");
	p.append(firstname);
	p.append("*");
	p.append(lastname);
	if ((hFile = _findfirst(p.c_str(), &fileinfo)) != -1) {
		do {
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib & _A_SUBDIR)) {
				continue;

				/*if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				getFiles(p.assign(folder).append("\\").append(fileinfo.name), files);*/
			}
			else {
				string temp = p.assign(folder).append(fileinfo.name);
				files.push_back(p.assign(folder).append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	return files;
}


bool input_assist(Mat im,map<string, string> main_ini, vector<assist_information> & assist_files,int assist_index)
{
	fstream assist_file_name(main_ini["assist_txt"]);
	if (!assist_file_name)
		return false;
	// 用于边缘检测时排除干扰
	Mat temp_im = im.clone();
	//
	assist_files.clear();
	string temp_name;
	getline(assist_file_name, temp_name);
	vector<double> temp;
	get_number(temp_name, temp);
	int ruler_number = temp[0];
	vector<assist_information> temp_assist_files;
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
		input_assist_image(main_ini["assist_image"], temp_assist_file);
		// 读取模板图片
		string temp_name = string(main_ini["template"].begin(), main_ini["template"].end() - 4)
			+ "_" + to_string((int)temp[2]) + string(main_ini["template"].end()-4, main_ini["template"].end());
		Mat template_image = imread(temp_name, IMREAD_GRAYSCALE);
		if (!template_image.data) {
			cout << "template path error!" << endl;
			return false;
		}
		temp_assist_file.base_image = template_image;
		temp_assist_file.length = temp[2];
		// roi
		vector<double> roi;
		getline(assist_file_name, temp_name);
		get_number(temp_name, roi);
		if (roi.size() != 4)
			break;
		temp_assist_file.roi = roi;
		// sub_roi
		vector<double> sub_roi;
		getline(assist_file_name, temp_name);
		get_number(temp_name, sub_roi);
		if (sub_roi.size() != 4)
			break;
		temp_assist_file.sub_roi = sub_roi;
		// point
		vector<vector<double>> temp_point;
		for (int n = 0; n < temp[temp.size() - 1]; ++n) {
			getline(assist_file_name, temp_name);
			vector<double> temp;
			get_number(temp_name, temp);
			temp_point.push_back(temp);
		}
		temp_assist_file.point = temp_point;
		temp_assist_file.water_number = 0;
		//
		temp_assist_files.push_back(temp_assist_file);
	}
	if (assist_index >= 0) {
		int i = 0;
		return true;
	}

	vector<Mat> d_image = cal_search_feature(im);
	vector<Feature> contours;
	Rect base_rect;
	Mat result;
	float score = 0, temp_score=0;
	Point temp_result_point, result_point;
	for(int i=0;i<temp_assist_files.size();++i) {
		assist_information temp = temp_assist_files[i];
		if (isgrayscale(im) ^ isgrayscale(temp.assist_image))
			continue;
		Mat im2 = temp.assist_image(
			Range(temp.sub_roi[1], temp.sub_roi[1] + temp.sub_roi[3]),
			Range(temp.sub_roi[0], temp.sub_roi[0] + temp.sub_roi[2])
		).clone();
		contours = make_template(im2, base_rect);
		find_object(im, d_image, im2, base_rect, contours, result_point, score);
		//
		result = draw_contours(im, contours, result_point, 3);
		if (assist_files.size() == temp.roi_order) {
			if (score < assist_files[temp.roi_order-1].correct_score)
				continue;
		}
		else {
			assist_files.push_back(temp);
		}
		if (score < match_t)
			continue;
		// 偏移量求解
		temp.base_point.x = result_point.x - base_rect.x - temp.sub_roi[0];
		temp.base_point.y = result_point.y - base_rect.y - temp.sub_roi[1];

		Mat homography = Mat::zeros(Size(3, 3), CV_32F);
		homography.at<float>(0, 0) = 1; homography.at<float>(1, 1) = 1; homography.at<float>(2, 2) = 1;
		homography.at<float>(0, 2) = result_point.x - base_rect.x;
		homography.at<float>(1, 2) = result_point.y - base_rect.y;
		vector<vector<double>> points = temp.point;
		for (auto &point : points) {
			double x = point[2] - temp.sub_roi[0];
			double y = point[3] - temp.sub_roi[1];
			point[2] = x * homography.at<float>(0, 0) + y * homography.at<float>(0, 1) + homography.at<float>(0, 2);
			point[3] = x * homography.at<float>(1, 0) + y * homography.at<float>(1, 1) + homography.at<float>(1, 2);
		}
		// 生成足够的点
		temp.point = points;
		temp.correct_score = score;
		if (assist_files.size() == temp.roi_order)
			assist_files[temp.roi_order-1] = temp;
		else if (assist_files.size() < temp.roi_order-1)
			assist_files.push_back(temp);
		else
			return false;
	}
}

bool get_number(string line_string, vector<double>& temp)
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
bool input_assist_image(string file_name, assist_information &assist_file)
{
	string temp = string(file_name.begin(), file_name.end() - 4) + "_" + to_string(assist_file.ref_index) + string(file_name.end() - 4, file_name.end());
	Mat temp_image = imread(temp, IMREAD_COLOR);
	if (!temp_image.data)
		return false;
	assist_file.assist_image = temp_image.clone();
	return true;
}
void compute_water_area(Mat im, vector<assist_information> &assist_files, string ref_name)
{
	for (auto &assist_file : assist_files) {
		// 
		get_parrallel_lines(assist_file);

		Mat mask = imread(ref_name,CV_LOAD_IMAGE_UNCHANGED);
		if (!mask.data) {
			cout << "no mask image!" << endl;
			return;
		}
		Mat image_rotate = get_ref_index(assist_file, mask);
		//
		if(assist_file.left_right_no_water)
			assist_file.water_line = get_water_line_seg(image_rotate, assist_file.length);
		else {
			assist_file.water_line = get_water_line_seg(image_rotate.colRange(image_rotate.cols/3, image_rotate.cols*2 / 3), assist_file.length);
		}
		if (assist_file.water_line < 0) {
			assist_file.parrallel_lines.clear();
			continue;
		}
		linetonumber(assist_file);
	}
}

void opt_assist_files(vector<assist_information>& assist_files)
{
	//for (int i = 0; i < assist_files.size(); ++i) {
	//	if (assist_files[i].parrallel_lines.size() < 1)
	//		continue;
	//	else
	//		temp_assist_files.push_back(assist_files[i]);
	//}
	//stable_sort(assist_files.begin(), assist_files.end(),
	//	[](assist_information a, assist_information b) {return a.correct_score > b.correct_score; });
	vector<bool> assist_file_flag(assist_files.size(), true);
	for (int i = 0; i < assist_files.size(); ++i) {
		if (!assist_file_flag[i])
			continue;
		if (assist_files[i].parrallel_lines.size() == 0) {
			assist_file_flag[i] = false;
			continue;
		}
		int x1 = 9999, x2 = 0;
		x1 = x1 < assist_files[i].parrallel_lines[0].x ? x1 : assist_files[i].parrallel_lines[0].x;
		x1 = x1 < assist_files[i].parrallel_lines[1].x ? x1 : assist_files[i].parrallel_lines[1].x;
		x2 = x2 > assist_files[i].parrallel_lines[2].x ? x2 : assist_files[i].parrallel_lines[2].x;
		x2 = x2 > assist_files[i].parrallel_lines[3].x ? x2 : assist_files[i].parrallel_lines[3].x;
		for (int j = i + 1; j < assist_files.size(); ++j) {
			if (!assist_file_flag[j])
				continue;
			for (int k = 0; k < 4; ++k) {
				if (assist_files[j].parrallel_lines.size() == 0) {
					assist_file_flag[j] = false;
					continue;
				}
				if (assist_files[j].parrallel_lines[k].x >= x1 &&
					assist_files[j].parrallel_lines[k].x <= x2) {
					if (assist_files[i].correct_score > assist_files[j].correct_score) {
						assist_file_flag[j] = false;
					}
					else {
						assist_file_flag[i] = false;
					}
					break;

				}
			}
		}
	}
	for (int i = 0; i < assist_file_flag.size(); ++i) {
		if (!assist_file_flag[i]) {
			assist_files[i].parrallel_left.clear();
			assist_files[i].parrallel_right.clear();
			assist_files[i].parrallel_lines.clear();
			assist_files[i].water_lines.clear();
		}
	}
}
bool isgrayscale(Mat im)
{
	Mat temp = im.clone();
	if (temp.channels() == 3) {
		temp.convertTo(temp, CV_8UC3);
		cv::Vec3b * data = temp.ptr<cv::Vec3b>(0);
		float num = 0;
		for (int i = 0; i < temp.total(); ++i) {
			int score = abs((data[i])[0] - (data[i])[1]);
			score += abs((data[i])[1] - (data[i])[2]);
			score += abs((data[i])[0] - (data[i])[2]);
			if (score <3)
				++num;
		}
		if (num / temp.total() < 0.8) {
			return false;
		}
		else {
			return true;
		}
		return false;
	}
	else {
		return true;
	}

}
bool isblank(Mat im, assist_information &assist_file)
{
	int n_length = assist_file.base_image.rows / assist_file.length *10;
	// 旋转校正 多部分 包含对原始影像进行矫正
	assist_information temp_assist_file = assist_file;
	temp_assist_file.wrap_image = Mat::zeros(Size(temp_assist_file.base_image.cols * 3,
		temp_assist_file.wrap_image.rows), CV_64F);
	for (int i = 0; i < temp_assist_file.point.size(); ++i) {
		(temp_assist_file.point[i])[0] += temp_assist_file.base_image.cols;
	}
	temp_assist_file.correct2poly = true;
	Mat image_rotate = correct_image(im, temp_assist_file);
	// 根据颜色去除
	Mat temp_im;
	cvtColor(image_rotate, temp_im, CV_BGR2GRAY);
	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	Mat hist;
	calcHist(&temp_im, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	float temp1 = 0, temp2 = 0;
	for (int i = 0; i < 40; ++i) {
		temp1 += hist.at<float>(i, 0);
		temp2 += hist.at<float>(histSize - i - 1, 0);
	}
	temp1 = temp1 / (temp_im.rows*temp_im.cols);
	temp2 = temp2 / (temp_im.rows*temp_im.cols);
	if (temp1 > 0.9 || temp2 > 0.9)
		return true;
	float num_zeros = 0;
	// 根据超出区域来
	for (int i = 0; i < assist_file.mask.total(); ++i) {
		Vec3s temp = *(assist_file.mask.ptr<Vec3s>(0) + i);
		if (temp[0] == temp[1] && temp[1] == temp[2] && temp[0] == 1024)
			num_zeros = num_zeros + 1;
	}
	num_zeros = num_zeros / assist_file.base_image.total();
	if (num_zeros > 0.15)
		return true;
	//
	assist_file.expand_wrap_image = image_rotate.clone();

	Mat left_image = image_rotate.colRange(0, assist_file.base_image.cols).clone();
	Mat base_image = image_rotate.colRange(assist_file.base_image.cols, assist_file.base_image.cols * 2).clone();
	Mat right_image = image_rotate.colRange(assist_file.base_image.cols * 2, assist_file.base_image.cols * 3).clone();
	assist_file.left_image = left_image.clone();
	assist_file.right_image = right_image.clone();
	if (!isgrayscale(im)) {

		//
		Mat temp1 = base_image.rowRange(0, n_length).clone();
		Mat temp2 = left_image.rowRange(0, n_length).clone();
		Mat temp3 = right_image.rowRange(0, n_length).clone();
		float score1, score2;
		Mat temp;
		matchTemplate(temp1, temp2, temp, CV_TM_SQDIFF_NORMED);
		score1 = temp.at<float>(0, 0);
		matchTemplate(temp1, temp3, temp, CV_TM_SQDIFF_NORMED);
		score2 = temp.at<float>(0, 0);
		if (score1 < 0.05 && score2 < 0.05)
			return true;

	}
	else {
	
		Mat temp = image_rotate.rowRange(0, n_length).clone();
		vector<Mat> splt;
		split(temp, splt);
		temp = splt[0].clone();
		float score = 0;
		for (int i = 0; i < temp.total(); ++i) {
			if (*(temp.ptr<uchar>(0) + i) > 160)
				score += 1;
		}
		score = score / temp.total();
		if (score > 0.7)
			return false;
		
		Mat temp1 = base_image.rowRange(0, n_length).clone();
		Mat temp2 = left_image.rowRange(0, n_length).clone();
		Mat temp3 = right_image.rowRange(0, n_length).clone();
		float score1, score2;
		matchTemplate(temp1, temp2, temp, CV_TM_SQDIFF_NORMED);
		score1 = temp.at<float>(0, 0);
		matchTemplate(temp1, temp3, temp, CV_TM_SQDIFF_NORMED);
		score2 = temp.at<float>(0, 0);
		if (score1 < 0.15 && score2 < 0.15)
			return true;

	}
	//if (!isgrayscale(im)) {
	//	temp_assist_file.wrap_image = Mat::zeros(Size(temp_assist_file.base_image.cols * 7,
	//		temp_assist_file.wrap_image.rows), CV_64F);
	//	for (int i = 0; i < temp_assist_file.point.size(); ++i) {
	//		(temp_assist_file.point[i])[0] += 2*temp_assist_file.base_image.cols;
	//	}
	//	Mat image_rotate = correct_image(im, temp_assist_file);
	//	assist_file.expand_wrap_image = image_rotate.clone();
	//}
	return false;
}
bool notall(Mat im, assist_information &assist_file)
{

	bool flag = false;
	int num_e = assist_file.length / 5;
	int n_length = assist_file.base_image.rows / (float)assist_file.length * 5;
	Mat left_image = assist_file.left_image;
	Mat right_image = assist_file.right_image;
	Mat temp_im = im.clone();
	if (!isgrayscale(temp_im)) {
		cvtColor(left_image, left_image, CV_BGR2GRAY);
		cvtColor(right_image, right_image, CV_BGR2GRAY);
		cvtColor(temp_im, temp_im, CV_BGR2GRAY);
	}


	Mat temp,temp1, temp2, temp3;
	for (int i = 0; i < num_e; ++i) {
		temp1 = temp_im.rowRange(i*n_length, (i + 1)*n_length);
		temp2 = left_image.rowRange(i*n_length, (i + 1)*n_length);
		temp3 = right_image.rowRange(i*n_length, (i + 1)*n_length);
		matchTemplate(temp1, temp2, temp, CV_TM_SQDIFF_NORMED);
		assist_file.scores1.push_back(temp.at<float>(0, 0));
		if (temp.at<float>(0, 0) < 0.1)
			flag = true;
		matchTemplate(temp1, temp3, temp, CV_TM_SQDIFF_NORMED);
		assist_file.scores2.push_back(temp.at<float>(0, 0));
		if (temp.at<float>(0, 0) < 0.1)
			flag = true;
	}
	return flag;
}



bool isTooHighLightInNight(Mat im, int &water_line, int &gray_value)
{
	int c = im.cols / 3;
	float n = 0;
	int temp_water_line = water_line;
	if (temp_water_line < 1)
		temp_water_line = im.rows-1;
	for(int i = 0;i< temp_water_line;++i)
		for (int j = c; j < 2 * c; ++j) {
			if (im.at<uchar>(i, j) > gray_value) {
				++n;
			}
		}
	n = n / c / temp_water_line;
	if (n < 0.4)
		return false;
	Mat temp = im(Range(0, temp_water_line), Range(c, c * 2));
	gray_value = TresholdOtsu(temp);
	int num = 0;
	int temp_n = 0;
	for (int i = 0; i < temp.total(); ++i) {
		if (*(temp.ptr<uchar>(0) + i) > gray_value) {
			num += *(temp.ptr<uchar>(0) + i);
			++temp_n;
		}
	}
	gray_value = num / temp_n -1;
	if (gray_value >= 230) {
		gray_value = 230;
		return true;
	}
	else {
		return false;
	}
}

Mat correct_image(Mat im, assist_information &assist_file)
{
	// 对原始图像进行处理
	//
	Mat result,map_x,map_y;
	map_coord(assist_file, map_x, map_y);// map_x,map_y float型数据
	im.convertTo(im, CV_16UC3);
	remap(im, result, map_x, map_y, CV_INTER_CUBIC, 0,Scalar{ 1024,1024,1024 });
	assist_file.mask = result;
	result.convertTo(result, CV_8UC3);
	im.convertTo(im, CV_8UC3);
	assist_file.wrap_image = result;
	// 将roi 即水尺区域的平行线保存
	// 存在一些问题 需要使用拟合的方式进行一定的处理
	Vec4f line_para;
	Point2d point1, point2;
	vector<Point2d> points;
	// 左侧线段
	//Mat test_im = Mat::zeros(Size(2000, 2000), CV_8UC3);
	//Mat temp_points1 = Mat::zeros(Size(2,assist_file.wrap_image.rows), CV_64F);
	for (int i = 0; i < assist_file.wrap_image.rows; ++i) {
		points.push_back(Point2d(map_x.at<float>(i, 0), map_y.at<float>(i, 0)));
		//temp_points1.at<double>(i, 0) = map_x.at<float>(i, 0);
		//temp_points1.at<double>(i, 1) = map_y.at<float>(i, 0);
		//circle(test_im, points[i], 1, Scalar(255, 0, 0));
		assist_file.parrallel_left.push_back(points[i]);
	}
	fitLine(points, line_para, CV_DIST_FAIR, 0, 1e-2, 1e-2);
	point1.y = 9999; point2.y = -9999;
	for (auto i : points) {
		float d = (i.x - line_para[2])*line_para[0] + (i.y - line_para[3])*line_para[1];
		float x = d * line_para[0] + line_para[2];
		float y = d * line_para[1] + line_para[3];
		if (point1.y > y) {
			point1.x = x;
			point1.y = y;
		}
		if (point2.y < y) {
			point2.x = x;
			point2.y = y;
		}
	}
	//line(test_im, point1, point2, Scalar(0, 255, 0));
	assist_file.parrallel_lines.push_back(point1);
	assist_file.parrallel_lines.push_back(point2);
	// 右侧线段
	points.clear();
	for (int i = 0; i < assist_file.wrap_image.rows; ++i) {
		points.push_back(Point2d(map_x.at<float>(i, assist_file.wrap_image.cols-1), map_y.at<float>(i, assist_file.wrap_image.cols - 1)));
		//circle(test_im, points[i], 1, Scalar(255, 0, 0));
		assist_file.parrallel_right.push_back(points[i]);
	}
	fitLine(points, line_para, CV_DIST_FAIR, 0, 1e-2, 1e-2);
	point1.y = 9999; point2.y = -9999;
	for (auto i : points) {
		float d = (i.x - line_para[2])*line_para[0] + (i.y - line_para[3])*line_para[1];
		float x = d * line_para[0] + line_para[2];
		float y = d * line_para[1] + line_para[3];
		if (point1.y > y) {
			point1.x = x;
			point1.y = y;
		}
		if (point2.y < y) {
			point2.x = x;
			point2.y = y;
		}
	}
	//line(test_im, point1, point2, Scalar(0, 255, 0));
	assist_file.parrallel_lines.push_back(point1);
	assist_file.parrallel_lines.push_back(point2);
	return result;
}

void map_coord(assist_information & assist_file, Mat & map_x, Mat & map_y, int base_x, int base_y)
{
	// 获取变换的矩阵
	Mat r = GeoCorrect2Poly(assist_file, true);
	Mat r_inv = GeoCorrect2Poly(assist_file, false);
	Mat point = vector2Mat(assist_file.point);
	double rms_1 = compute_rms(point.colRange(0, 2), point.colRange(2, 4), r);
	double rms_2 = compute_rms(point.colRange(2, 4), point.colRange(0, 2), r_inv);
	assist_file.r = r;
	assist_file.r_inv = r_inv;
	assist_file.rms_1 = rms_1;
	assist_file.rms_2 = rms_2;
	//
	Mat temp1 = Mat::zeros(assist_file.wrap_image.size(), CV_64F);
	for (int i = 0; i < temp1.cols; ++i) {
		temp1.col(i).setTo(i+ base_x);
	}
	temp1 = temp1.reshape(0,(int)temp1.total());
	Mat temp2 = Mat::zeros(assist_file.wrap_image.size(), CV_64F);
	for (int i = 0; i < temp2.rows; ++i) {
		temp2.row(i).setTo(i+ base_y);
	}
	temp2 = temp2.reshape(0, (int)temp2.total());
	Mat temp;
	hconcat(temp1, temp2, temp);
	temp = compute_point(temp, r);
	map_x = temp.col(0).clone();
	map_x = map_x.reshape(0, assist_file.wrap_image.rows);
	map_x.convertTo(map_x, CV_32F);
	map_y = temp.col(1).clone();
	map_y = map_y.reshape(0, assist_file.wrap_image.rows);
	map_y.convertTo(map_y, CV_32F);
	// 
	//if (assist_file.correct_flag) {
	//	Mat temp = Mat::zeros(Size(2,assist_file.correct_point.size()), CV_64F);
	//	for (int i = 0; i < assist_file.correct_point.size();++i) {
	//		temp.at<double>(i, 0) = (assist_file.correct_point[i])[0];
	//		temp.at<double>(i, 1) = (assist_file.correct_point[i])[1];
	//	}
	//	temp= compute_point(temp, r_inv);
	//	assist_file.correct_line = 0;
	//	for (int i = 0; i < temp.rows; ++i) {
	//		assist_file.correct_line = assist_file.correct_line > temp.at<double>(i, 1) ?
	//			assist_file.correct_line : temp.at<double>(i, 1);
	//	}
	//}

}

Mat GeoCorrect2Poly(assist_information assist_file, bool flag)
{
	int n = (int)assist_file.point.size();
	Mat point;
	if (flag) 
		point = vector2Mat(assist_file.point);
	else{
		vector<vector<double>> temp= assist_file.point;
		swap_point(temp);
		point = vector2Mat(temp);
	}
	Mat X = point.col(0).clone();
	Mat Y = point.col(1).clone();
	Mat XY = X.mul(Y), XX = X.mul(X), YY = Y.mul(Y);
	Mat M; 

	if (assist_file.correct2poly) {
		M = Mat::zeros(Size(6, n), CV_64F);
		M.col(0).setTo(1);
		X.copyTo(M.col(1));
		Y.copyTo(M.col(2));
		XY.copyTo(M.col(3));
		XX.copyTo(M.col(4));
		YY.copyTo(M.col(5));
	}
	else {
		M = Mat::zeros(Size(4, n), CV_64F);
		M.col(0).setTo(1);
		X.copyTo(M.col(1));
		Y.copyTo(M.col(2));
		XY.copyTo(M.col(3));

	}
	Mat A, B;
	Mat temp = M.t()*M;
	A = (M.t()*M).inv()*M.t()*point.col(2);
	B = (M.t()*M).inv()*M.t()*point.col(3);
	Mat result = Mat::zeros(Size(2, 6), CV_64F);
	for (int i = 0; i < A.rows; ++i) {
		result.at<double>(i, 0) = A.at<double>(i, 0);
		result.at<double>(i, 1) = B.at<double>(i, 0);
	}
	return result;
}

Mat compute_point(Mat point, Mat r)
{
	Mat M = Mat::ones(Size(1, point.rows), CV_64F);
	Mat X = point.col(0).clone() , Y = point.col(1).clone();
	hconcat(M, X, M);
	hconcat(M,Y, M);
	hconcat(M, X.mul(Y), M);
	hconcat(M, X.mul(X), M);
	hconcat(M, Y.mul(Y), M);
	return M * r;
}

double compute_rms(Mat base_point, Mat wrap_point, Mat r)
{
	double rms = 0;
	Mat temp_point = compute_point(base_point, r) - wrap_point;
	pow(temp_point , 2, temp_point);
	for (int i = 0; i < temp_point.total(); ++i)
		rms += *(temp_point.ptr<double>(0) + i);
	rms = sqrt(rms / temp_point.rows);
	return rms;
}

float get_water_line_night(Mat im,assist_information & assist_file)
{
	int water_line = -1;
	int add_rows = assist_file.wrap_image.rows- assist_file.base_image.rows;
	vector<Mat> splt;
	split(assist_file.expand_wrap_image, splt);
	Mat temp = splt[0].clone();
	int n_length = 3 * (double)assist_file.base_image.rows / assist_file.length;
	int c = temp.cols / 3;
	if (!assist_file.left_right_no_water) {
		int temp_t = TresholdOtsu(temp.colRange(c, c * 2));
		threshold(temp, temp, temp_t, 255, CV_THRESH_BINARY);
		water_line = get_water_line_seg(temp.colRange(c, c * 2), assist_file.length, add_rows);
		int gray_value = 230;
		if (isTooHighLightInNight(splt[0], water_line, gray_value)) {
			threshold(splt[0], temp, gray_value, 255, CV_THRESH_BINARY);
			water_line = get_water_line_seg(temp.colRange(c, c * 2), assist_file.length, add_rows);
		}
	}
	else {
		threshold(temp, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		Mat result;
		hconcat(temp.colRange(0.5*c,c), temp.colRange(c*2, c*2.5), result);
		int water_line2 = -1;
		water_line = get_water_line_seg(result, assist_file.length, add_rows);
		
		int gray_value = 230;
		if (isTooHighLightInNight(splt[0], water_line, gray_value)) {
			int temp_t = TresholdOtsu(temp);
			threshold(temp, temp, temp_t, 255, CV_THRESH_BINARY);
			if (left_right_water(temp, n_length)) {
				water_line = get_water_line_seg(temp.colRange(c, 2 * c), assist_file.length, add_rows);
			}
			else {

				hconcat(temp.colRange(0.5*c, c), temp.colRange(c * 2, c*2.5), result);
				water_line = get_water_line_seg(result, assist_file.length, add_rows,0.05);
			}
		}
	}



	return water_line;
}

float get_water_line_night_local(Mat im, assist_information & assist_file)
{
	int water_line = -1;
	int add_rows = assist_file.wrap_image.rows - assist_file.base_image.rows;
	vector<Mat> splt;
	split(assist_file.expand_wrap_image, splt);
	int c = splt[0].cols / 3;
	Mat temp,temp_im;
	int n_length = 5 * (double)assist_file.base_image.rows / assist_file.length;
	// 
	temp_im = splt[0].clone();
	threshold(temp_im, temp, 0, 255, CV_THRESH_BINARY| CV_THRESH_OTSU);
	float temp_t1 =0, temp_t2 = 2,temp_n =0;
	for (int i = 0; i < temp.rows; ++i)
		for (int j = 0; j < temp.cols; ++j){
			if (temp.at<uchar>(i, j) > 100) {
				temp_t1 += temp_im.at<uchar>(i, j);
				temp_n++;
			} 
			else
				temp_t2 += temp_im.at<uchar>(i, j);
		}
	temp_t1 = temp_t1 / temp_n;
	temp_t2 = temp_t2 / (temp_im.total()-temp_n);
	int water_line1 = get_water_line_seg(temp.colRange(c,c*2), assist_file.length, add_rows);
	//
	temp_im = splt[0].colRange(c, c * 2).clone();
	
	vector<int> score;
	for (int i = 0; i < temp_im.rows;) {
		int y1 = i;
		int y2 = i + 10 * n_length;
		if (y2 > temp_im.rows) {
			if (y1 < temp_im.rows) {
				threshold(temp_im.rowRange(y1, temp_im.rows), temp_im.rowRange(y1, temp_im.rows), temp_t1, 255, CV_THRESH_BINARY);
			}
			break;
		}
		Mat temp1 = temp_im(Range(y1, y2), Range(0, c)).clone();
		Mat temp2;
		threshold(temp1, temp2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		float n1 = 0, n2 = 0, n = 0;
		for (int k = 0; k < temp2.total(); ++k) {
			if (*(temp2.ptr<uchar>(0) + k) > 100) {
				n++;
				n1 += *(temp1.ptr<uchar>(0) + k);
			}
			else
				n2 += *(temp1.ptr<uchar>(0) + k);
		}
		float score1 =  n1 / n,score2 = n2 / (temp2.total() - n),score = (n1+n2)/temp2.total();
		if ((abs(score1 - score) > 20 || abs(score2 - score) > 20) &&score < 150&&score1<200) {
			temp1 = temp_im(Range(y1, y2), Range(0, c*0.5));
			temp2 = temp_im(Range(y1, y2), Range(c*0.5, c));
			threshold(temp1, temp1, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			threshold(temp2, temp2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		}
		else 
			temp.colRange(c, c * 2).rowRange(y1, y2).copyTo(temp_im.rowRange(y1, y2)); 
		//
		i = y2;
	}
	temp_im.rowRange(temp_im.rows - add_rows, temp_im.rows).setTo(0);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 10));
	Mat out = temp_im.clone();
	dilate(out, out, element);
	erode(out, out, element);

	int water_line2 = get_water_line_seg(out, assist_file.length, add_rows);

	return water_line1>water_line2?water_line1:water_line2;
}

bool left_right_water(Mat gc_im, int length)
{
	vector<Mat> splt;
	split(gc_im, splt);
	Mat im = splt[0].clone();
	Mat temp;
	threshold(im, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	int n_length = 5 * im.rows / (float)length;
	int c = im.cols / 3;
	float num = 0;
	for (int i = 0; i < n_length; ++i) {
		int temp_num1 = 0,temp_num2 = 0;
		for (int j = 0; j < 1.5*c; ++j) {
			if (temp.at<uchar>(i, j) > 100)
				++temp_num1;
		}
		for (int j = 1.5*c; j < 3*c; ++j) {
			if (temp.at<uchar>(i, j) > 100)
				++temp_num2;
		}
		if ((temp_num1 > (0.6 * c))&& (temp_num2 > (0.6 * c)))
			++num;
	}
	if (num > 0.4*n_length)
		return false;
	else
		return true;


}

int get_water_line_seg(Mat im, int length, int add_rows, float scale)
{
	int n_length = 5 * (double)im.rows / length;
	int c = im.cols;
	bool ref_flag;
	//
	vector<double> im_score(im.rows, 0);
	for (int i = 0; i < im.rows; ++i) {
		int n = 0;
		for (int j = 0; j < im.cols; ++j) {
			if (im.at<uchar>(i, j) > 100)
				++n;
		}
		im_score[i] = n;
	}
	//
	ref_flag = false;
	int temp_n = 0;
	for (int i = 0; i < n_length; ++i) {
		if (im_score[i] < scale*im.cols)
			++temp_n;
	}
	if (temp_n > 0.8*n_length)
		ref_flag = true;
	if (ref_flag) {
		return -1;
	}
	//
	int seg_line = im.rows;
	for (int i = 0; i < im.rows - n_length; ++i) {
		if (im_score[i] < scale*c) {
			float temp_score = 0;
			for (int j = 0; j < n_length; ++j) {
				if (im_score[i + j] < scale*c) {
					temp_score++;
				}
			}
			if (temp_score > 0.8*n_length) {
				seg_line = i;
				break;
			}
			else {
				i = i + 0.5*n_length;
			}
		}
	}
	if (abs(im.rows - seg_line) < 5)
		return seg_line;
	for (int i = seg_line; i < im.rows; ++i) {
		im_score[i] = 0;
	}
	//
	vector<double> div1;
	vector<double> div2;
	for (int i = 0; i < im.rows; ++i) {
		if (i < n_length - 1 || i >= im.rows - n_length) {
			div1.push_back(0);
			div2.push_back(0);
			continue;
		}
		int n1 = 0;
		for (int j = i - n_length + 1; j <= i; ++j) {
			n1 += im_score[j];
			if (im_score[j] == 0)
				n1 -= 0.2 *c;
		}
		int n2 = 0;
		int n3 = 0;
		for (int j = i + 1; j <= i + n_length; ++j) {
			n2 += im_score[j];
			if (im_score[j] == 0) {
				n3 -= 0.2 *c;
			}
			else {
				n3 += im_score[j];
			}
		}
		div1.push_back(n1 - n2);
		div2.push_back(n1 - n3);
	}
	int water_line1 = 0;
	int water_line2 = 0;
	float score = 0;
	for (int i = 0; i < div1.size(); ++i) {
		if (score < div1[i]) {
			score = div1[i];
			water_line1 = i;
		}
	}
	score = 0;
	for (int i = 0; i < div2.size(); ++i) {
		if (score < div2[i]) {
			score = div2[i];
			water_line2 = i;
		}
	}
	if (water_line2 > im.rows - add_rows)
		return water_line2;
	// 判断是不是过长
	int n_flag = 0;
	for (int i = water_line2; i < water_line2 + add_rows; ++i) {
		if (im_score[i]< 0.5*c)
			++n_flag;
	}
	if (n_flag < 0.4*add_rows)
		return im.rows - add_rows + 1;
	else
		return water_line2;

}

void save_file(Mat im, vector<assist_information> assist_files, map<string, string> main_ini)
{
	//结果保存
	Mat result = im.clone();
	vector<vector<float>> temp_water_number;
	for (int i = 0; i < assist_files.size(); ++i) {

		assist_information temp = assist_files[i];
		// 画出水尺区域线
		if (temp.parrallel_lines.size() == 4) {
			line(result, temp.parrallel_lines[0], temp.parrallel_lines[1], Scalar(255, 0, 0), 2);
			line(result, temp.parrallel_lines[2], temp.parrallel_lines[3], Scalar(255, 0, 0), 2);		
		}
		if (temp.water_lines.size() == 4){
			// 画出水线
			line(result, Point2d(temp.water_lines[0], temp.water_lines[1]), Point2d(temp.water_lines[2], temp.water_lines[3]), Scalar(0, 255, 0), 2);
			// 画出数值
			ostringstream ss;
			ss << round(temp.water_number * 10) / 10;
			string text = ss.str() + "cm";
			int font_face = cv::FONT_HERSHEY_COMPLEX;
			double font_scale = 1;
			int thickness = 2;
			//获取文本框的长宽 
			int baseline;
			Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
			Point2d temp_point;
			temp_point.x = (temp.water_lines[0] + temp.water_lines[2]) / 2;
			temp_point.y = (temp.water_lines[1] + temp.water_lines[3]) / 2;
			if (temp_point.x < result.cols / 2)
				temp_point.x = temp_point.x + 30;
			else
				temp_point.x = temp_point.x - 30 - text_size.width;
			putText(result, text, temp_point, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
		}

	}
	// 读写图像
	cv::imwrite(main_ini["result_image"], result);
	// 读写文件
	ofstream file(main_ini["result_txt"]);
	for (int i = 0; i < assist_files.size(); ++i) {
		assist_information temp 
			 = assist_files[i];
		file << "No=";
		file << (i + 1) << ";" << endl;
		file << "WaterLevel=";
		if (temp.water_lines.size() != 4|| temp.parrallel_lines.size() != 4)
			file << ";" << endl;
		else
			file << fixed << setprecision(1) << round(temp.water_number * 10) / 10 << ";" << endl;
		file << "GuageArea=";
		for (int j = 0; j < temp.parrallel_lines.size(); ++j) {
			file << fixed << setprecision(2) << temp.parrallel_lines[j].x;
			file << ",";
			file << fixed << setprecision(2) << temp.parrallel_lines[j].y;
			file << ",";
		}
		file << ";";
		file << endl;
		file << "WaterLine=";
		for (int j = 0; j < temp.water_lines.size(); ++j) {
			file << fixed << setprecision(2) << temp.water_lines[j];
			file << ",";
		}
		file << ";"<<endl;
	}
	file.close();
}

void save_maskfile(vector<assist_information> assist_files, map<string, string> main_ini)
{
	stable_sort(assist_files.begin(), assist_files.end(),
		[](assist_information a, assist_information b) {return a.roi_order < b.roi_order; });
	// 读写文件
	string save_name(main_ini["mask_image"].begin(), main_ini["mask_image"].end() - 4);
	save_name = save_name + ".txt";
	ofstream file(save_name);
	for (int i = 0; i < assist_files.size(); ++i) {
		assist_information assist_file = assist_files[i];
		file << assist_file.ref_index << ";";
		file << endl;
		//
		file << fixed << setprecision(2) << assist_file.base_point.x << "," ;
		file << fixed << setprecision(2) << assist_file.base_point.y << ";" ;
		file << endl;
		//
		file << assist_file.roi[0] << ",";
		file << assist_file.roi[1] << ",";
		file << assist_file.roi[2] << ",";
		file << assist_file.roi[3] << ";";
		file << endl;
	}

	file.close();
}

vector<vector<double>> part_row_point(vector<vector<double>> point, int r1, int r2)
{
	vector<vector<double>> result;
	int m = (int)point.size();
	int n = m == 0 ? 0: (int)point[0].size();
	if (m == 0 || n == 0)
		return result;
	if (r2 < r1)
		swap(r1, r2);
	r1 = r1 < m ? r1 : m;
	r2 = r2 < m ? r2 : m;
	for (int i = r1; i < r2; ++i)
		result.push_back(point[i]);
	return result;
}

vector<vector<double>> part_col_point(vector<vector<double>> point, int c1, int c2)
{
	vector<vector<double>> result;
	int m = (int)point.size();
	int n = m == 0 ? 0 : (int)point[0].size();
	if (m == 0 || n == 0)
		return result;
	if (c2 < c1)
		swap(c1, c2);
	c1 = c1 < n ? c1 : n;
	c2 = c2 < n ? c2 : n;
	for (auto i : point) 
		result.push_back(vector<double>{i.begin()+c1, i.begin() + c2});
	return result;
}

vector<vector<double>> swap_point(vector<vector<double>> &data)
{
	for (auto &i : data) {
		vector<double> temp = i;
		i[0] = temp[2];
		i[1] = temp[3];
		i[2] = temp[0];
		i[3] = temp[1];
	}
	return data;
}

Mat vector2Mat(vector<vector<double>> data)
{
	int m = 0, n = 0;
	m = (int)data.size();
	if (m==0)
		return Mat();
	n = (int)data[0].size();
	if (n == 0)
		return Mat();
	Mat result = Mat::zeros(Size(n, m), CV_64F);
	for (int i =0;i<m;++i)
		for (int j = 0; j < n; ++j) {
			result.at<double>(i, j) = (data[i])[j];
		}
	return result;
}

vector<vector<double>> Mat2vector(Mat data)
{
	int m = data.rows, n = data.cols;
	vector<vector<double>> result(m);
	for (int i = 0; i < m; ++i) {
		vector<double> temp(n);
		for (int j = 0; j < n; ++j) {
			temp[j] = data.at<double>(i, j);
		}
		result[i] = temp;
	}
	return result;
}

void get_parrallel_lines(assist_information &assist_file)
{
	vector<vector<double>> Points(assist_file.point.begin(), assist_file.point.end() - 2);
	// 拟合直线
	vector<Point2f> temp_point;
	for (int i = 0; i < Points.size(); ++i) {
		temp_point.push_back(Point2f(Points[i][2], Points[i][3]));
	}
	Vec4f line_para;
	fitLine(temp_point, line_para, CV_DIST_L2, 0, 1e-2, 1e-2);
	for (int i = 0; i < temp_point.size(); ++i) {
		float x = 0, y = 0;
		double d = (temp_point[i].x - line_para[2])*line_para[0] + (temp_point[i].y - line_para[3])*line_para[1];
		x = d * line_para[0] + line_para[2];
		y = d * line_para[1] + line_para[3];
		temp_point[i].x = x;
		temp_point[i].y = y;
		Points[i][2] = temp_point[i].x;
		Points[i][3] = temp_point[i].y;
		Points[i].push_back(d);
	}
	stable_sort(Points.begin(), Points.end(),
		[](vector<double> a, vector<double> b) {return a[4] > b[4]; });
	// 插值
	vector<double> first = interpolate_point(Points[0], Points[1], 0);
	vector<double> last = interpolate_point(Points[Points.size() - 2], Points[Points.size() - 1], assist_file.base_image.rows - 1);
	if (first.size() != 0) {
		first[2] = (first[2] - line_para[2])*line_para[0] + (first[3] - line_para[3])*line_para[1] * line_para[0] + line_para[2];
		first[3] = (first[2] - line_para[2])*line_para[0] + (first[3] - line_para[3])*line_para[1] * line_para[1] + line_para[3];
		Points.insert(Points.begin(), first);
	}
	if (last.size() != 0) {
		last[2] = (last[2] - line_para[2])*line_para[0] + (last[3] - line_para[3])*line_para[1] * line_para[0] + line_para[2];
		last[3] = (last[2] - line_para[2])*line_para[0] + (last[3] - line_para[3])*line_para[1] * line_para[1] + line_para[3];
		Points.insert(Points.end(), last);
	}

	assist_file.line_para = line_para;

	// 进行平移
	first = Points[0];
	last = Points[Points.size()-1];
	assist_file.parrallel_lines.push_back(Point2d(first[2],first[3]));
	assist_file.parrallel_lines.push_back(Point2d(last[2], last[3]));
	assist_file.parrallel_lines.push_back(Point2d(first[2], first[3]));
	assist_file.parrallel_lines.push_back(Point2d(last[2], last[3]));

	vector<double> temp;
	Point2d base_point;
	temp = assist_file.point[assist_file.point.size() - 2];
	base_point = Point2d(temp[2],temp[3]);
	change_point_by_line(assist_file.parrallel_lines[0], assist_file.parrallel_lines[1], base_point);

	temp = assist_file.point[assist_file.point.size() - 1];
	base_point = Point2d(temp[2], temp[3]);
	change_point_by_line(assist_file.parrallel_lines[2], assist_file.parrallel_lines[3], base_point);
	//
	assist_file.point = Points;
	//


	return;
}

vector<double> interpolate_point(vector<double>&point1, vector<double> &point2, int y)
{
	double x1, y1 = y, x2, y2;
	double k = (y1 - point2[1]) / (point1[1] - point2[1]);
	x1 = k * (point1[0] - point2[0]) + point2[0];
	x2 = k * (point1[2] - point2[2]) + point2[2];
	y2 = k * (point1[3] - point2[3]) + point2[3];
	vector<double> result;
	result.push_back(x1);
	result.push_back(y1);
	result.push_back(x2);
	result.push_back(y2);
	if (abs(y - point1[1]) < 3) {
		point1 = result;
		return vector<double>();
	}
	if (abs(y - point2[1]) < 3) {
		point2 = result;
		return vector<double>();
	}
	return result;
}
Point2d change_point_by_line(Point2d &first, Point2d &last, Point2d point)
{
	double d2 = (first.x - last.x)*(first.x - last.x) + (first.y - last.y)*(first.y - last.y);
	double k = ((point.x - last.x)*(first.x - last.x) + (point.y - last.y)*(first.y - last.y)) / d2;
	Point2d result;
	result.x = last.x + k * (first.x - last.x);
	result.y = last.y + k * (first.y - last.y);
	first = first - result + point;
	last = last - result + point;
	return result;
}

Mat get_ref_index(assist_information& assist_file, Mat mask)
{
	Mat line1(2, 4, CV_32F), line2(2, 4, CV_32F);
	Mat temp(2, 4, CV_32F);
	temp.setTo(0);
	temp.at<float>(0, 0) = assist_file.parrallel_lines[0].x;
	temp.at<float>(1, 0) = assist_file.parrallel_lines[0].y;
	temp.at<float>(0, 1) = assist_file.parrallel_lines[1].x;
	temp.at<float>(1, 1) = assist_file.parrallel_lines[1].y;
	line1 = temp.clone();
	temp.setTo(0);
	temp.at<float>(0, 0) = assist_file.parrallel_lines[2].x;
	temp.at<float>(1, 0) = assist_file.parrallel_lines[2].y;
	temp.at<float>(0, 1) = assist_file.parrallel_lines[3].x;
	temp.at<float>(1, 1) = assist_file.parrallel_lines[3].y;
	line2 = temp.clone();
	// 旋转原始影像
	float angle = atan(-assist_file.line_para[1]/ assist_file.line_para[0]) * 180 / atan(1) / 4;
	Point2d center(mask.cols / 2.0, mask.rows / 2.0);
	Mat r = getRotationMatrix2D(center, 90 - angle, 1.0);
	Rect bbox = RotatedRect(center, mask.size(), 90 - angle).boundingRect();
	bbox.width += 10000;
	bbox.height += 10000;
	r.at<double>(0, 2) += bbox.width / 2.0 - center.x;
	r.at<double>(1, 2) += bbox.height / 2.0 - center.y;
	Mat image_rotate;
	cv::warpAffine(mask, image_rotate, r, bbox.size(),CV_INTER_NN,0);
	// 截取大区域
	r.convertTo(r, CV_32F);
	for (int j = 0; j < 2; ++j) {
		Mat temp;
		line1.col(j).copyTo(temp);
		temp = r.colRange(0, 2) * temp + r.colRange(2, 3);
		temp.copyTo(line1.col(j + 2));
		line2.col(j).copyTo(temp);
		temp = r.colRange(0, 2) * temp + r.colRange(2, 3);
		temp.copyTo(line2.col(j + 2));
	}
	double temp1, temp2;
	line1.colRange(2, 4).copyTo(temp.colRange(0, 2));
	line2.colRange(2, 4).copyTo(temp.colRange(2, 4));
	double r1 = temp.at<float>(1, 0);
	double r2 = temp.at<float>(1, 1);
	double c1 = temp.at<float>(0, 0);
	double c2 = temp.at<float>(0, 2);
	if (r1 > r2)
		swap(r1, r2);
	if (c1 > c2)
		swap(c1, c2);
	//
	assist_file.r = Mat::zeros(Size(3, 3), CV_32FC1);
	r.copyTo(assist_file.r.rowRange(0, 2));
	assist_file.r.at<float>(2, 2) = 1;
	assist_file.r_inv = assist_file.r.inv();
	//
	double temp_c = c2 - c1;
	c1 = c1 - temp_c;
	c2 = c2 + temp_c;
	vector<int> rect;
	rect.push_back(r1);
	rect.push_back(2*r2-r1);
	rect.push_back(c1);
	rect.push_back(c2);
	assist_file.rect = rect;

	Mat data = image_rotate(Range(rect[0], rect[1]), Range(rect[2],rect[3]));
	return data;
}

double linetonumber(assist_information& assist_file)
{
	Point2d base_point(0, assist_file.water_line);
	base_point.x = base_point.x + assist_file.rect[2];
	base_point.y = base_point.y + assist_file.rect[0];
	Mat r_inv = assist_file.r_inv;
	Point2d wrap_point;
	wrap_point.x = r_inv.at<float>(0, 0)*base_point.x + r_inv.at<float>(0, 1)*base_point.y + r_inv.at<float>(0, 2);
	wrap_point.y = r_inv.at<float>(1, 0)*base_point.x + r_inv.at<float>(1, 1)*base_point.y + r_inv.at<float>(1, 2);

	Vec4d  line_para = assist_file.line_para;
	float d = (wrap_point.x - line_para[2])*line_para[0] + (wrap_point.y - line_para[3])*line_para[1];
	wrap_point.x = d * line_para[0] + line_para[2];
	wrap_point.y = d * line_para[1] + line_para[3];
	Mat result = assist_file.assist_image;
	line(result, wrap_point, wrap_point, Scalar(255, 0, 0), 2);
	vector<vector<double>> point = assist_file.point;
	for (int i = point.size() - 2; i >=0 ; --i) {
		Point2d first, last;
		first.x = point[i+1][2];
		first.y = point[i+1][3];
		last .x = point[i][2];
		last.y = point[i][3];
		double d2 = (first.x - last.x)*(first.x - last.x) + (first.y - last.y)*(first.y - last.y);
		double k = ((wrap_point.x - last.x)*(first.x - last.x) + (wrap_point.y - last.y)*(first.y - last.y)) / d2;
		
		if (k >=-0.1) {
			if (k > 1)
				k = 1.01;
			assist_file.water_number = k * (point[i + 1][1] - point[i][1]) + point[i][1];
			assist_file.water_number = assist_file.water_number * assist_file.length/ assist_file.base_image.rows;
			wrap_point.x = last.x + k * (first.x - last.x);
			wrap_point.y = last.y + k * (first.y - last.y);
			Point2d temp_point;
			//  水线
			first = assist_file.parrallel_lines[0];
			last = assist_file.parrallel_lines[1];
			temp_point = change_point_by_line(first, last, wrap_point);
			assist_file.water_lines.push_back(temp_point.x);
			assist_file.water_lines.push_back(temp_point.y);
			first = assist_file.parrallel_lines[2];
			last = assist_file.parrallel_lines[3];
			temp_point = change_point_by_line(first, last, wrap_point);
			assist_file.water_lines.push_back(temp_point.x);
			assist_file.water_lines.push_back(temp_point.y);
			assist_file.water_number = assist_file.length - assist_file.water_number;
			return assist_file.water_number;
		}
	}
	return -1;
}


Feature::Feature()
{
}

Feature::Feature(int x, int y, float d_x, float d_y, bool flag) : x(x), y(y), dx(d_x), dy(d_y), flag(flag)
{
}

Feature::~Feature(void)
{
}

Feature & Feature::operator=(Feature & feature)
{
	x = feature.x;
	y = feature.y;
	dx = feature.dx;
	dy = feature.dy;
	return *this;
}



vector<Feature> make_template(Mat image, Rect &base_rect)
{
	// 图像转灰度
	Mat template_gray_image;
	if (image.channels() == 3) {
		vector<Mat> splt;
		split(image, splt);
		template_gray_image = splt[0].clone();
	}
	else
		template_gray_image = image.clone();
	// 初步求出边缘
	GaussianBlur(template_gray_image, template_gray_image, cvSize(3, 3), 0);
	int bestLowThresh = 0, bestHighThresh = 0;
	int threshold_value = max(25, TresholdOtsu(template_gray_image));
	GetImageThreshold(template_gray_image, bestLowThresh, bestHighThresh);
	Mat edge;
	Canny(template_gray_image, edge, bestLowThresh, bestHighThresh);
	// 边缘筛选 并计算潜在轮廓点梯度
	int num = sum(edge)[0] / 255;
	int step = max(4, num / MAXFEANUM);
	vector<vector<Point>> contours_points;
	findContours(edge, contours_points, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	// 对轮廓排序
	stable_sort(contours_points.begin(), contours_points.end(),
		[](vector<Point> a, vector<Point> b) {return a[0].y < b[0].y; });
	//
	int frameStride = template_gray_image.step;
	uchar *srcPtr;
	vector<Feature> temp_contours;
	vector<Feature> contours;
	step = 2;
	for (size_t contourIdx = 0; contourIdx < contours_points.size(); contourIdx++)
	{
		int pointNum = contours_points[contourIdx].size();
		if (pointNum < 5)
			if (contours_points.size() > 2)
				continue;
		for (size_t pointIdx = 0; pointIdx < pointNum; pointIdx += step)
		{
			if (contours_points[contourIdx][pointIdx].x == 0
				|| contours_points[contourIdx][pointIdx].x == template_gray_image.cols - 1
				|| contours_points[contourIdx][pointIdx].y == 0
				|| contours_points[contourIdx][pointIdx].y == template_gray_image.rows - 1)
				continue;
			int x = contours_points[contourIdx][pointIdx].x;
			int y = contours_points[contourIdx][pointIdx].y;//原轮廓点（模板图像坐标系）
			srcPtr = (uchar*)template_gray_image.data + y * frameStride + x;
			float dx, dy;
			int	sumX = *(srcPtr - frameStride - 1) - *(srcPtr - frameStride + 1)
				+ *(srcPtr - 1) * 2 - *(srcPtr + 1) * 2
				+ *(srcPtr + frameStride - 1) - *(srcPtr + frameStride + 1);
			int sumY = *(srcPtr - frameStride - 1) + *(srcPtr - frameStride) * 2 + *(srcPtr - frameStride + 1)
				- *(srcPtr + frameStride - 1) - *(srcPtr + frameStride) * 2 - *(srcPtr + frameStride + 1);
			int grad = sumX * sumX + sumY * sumY;
			double sqrtgrad = sqrt(grad);
			if (sqrtgrad > threshold_value) {
				dx = sumX / sqrtgrad;
				dy = sumY / sqrtgrad;
				//if (y>32)
				//	contours.emplace_back(Feature(x, y, dx, dy,true));
				//else
				contours.emplace_back(Feature(x, y, dx, dy, false));
			}
			else
				continue;

		}

	}

	// 得到轮廓

	base_rect = GetBoundingRectangle(contours);
	// 处理一下
	template_gray_image = template_gray_image(Range(base_rect.y, base_rect.y + base_rect.height),
		Range(base_rect.x, base_rect.x + base_rect.width));
	for (int i = 0; i < contours.size(); ++i) {
		contours[i].x = contours[i].x - base_rect.x;
		contours[i].y = contours[i].y - base_rect.y;
	}
	// 赋值
	Mat temp_result = draw_contours(template_gray_image, contours);
	return contours;
}
void find_object(Mat search_image, vector<Mat> merge_im,
	Mat template_image, Rect base_rect, vector<Feature> contours,
	Point & result_point, float& score)
{
	// 图像转灰度处理
	Mat gray_image, template_gray_image;
	if (search_image.channels() == 3) {
		vector<Mat> splt;
		split(search_image, splt);
		gray_image = splt[0].clone();
	}
	//cvtColor(search_image, gray_image, CV_BGR2GRAY);
	else
		gray_image = search_image.clone();
	if (template_image.channels() == 3) {
		vector<Mat> splt;
		split(template_image, splt);
		template_gray_image = splt[0].clone();
	}
	//		cvtColor(template_image, template_gray_image, CV_BGR2GRAY);
	else
		template_gray_image = template_image.clone();
	// 
	if (template_gray_image.rows > gray_image.rows ||
		template_gray_image.cols > gray_image.cols)
		return;
	score = 0;
	double *m_dxImg = merge_im[0].ptr<double>(0);
	double *m_dyImg = merge_im[1].ptr<double>(0);
	int width = gray_image.cols;

	//计算相似度


	int Match_Greediness = 1;
	Feature feature;
	int x0, y0;
	float similarity = 0;
	Rect searchArea = Rect(0, 0, gray_image.cols - template_gray_image.cols,
		gray_image.rows - template_gray_image.rows);
	float min_score = 0.2;



	for (int i = searchArea.y; i < searchArea.y + searchArea.height / 2; i++)
	{
		for (int j = searchArea.x; j < searchArea.x + searchArea.width; j++)
		{
			similarity = 0.0;
			for (int feaIdx = 0; feaIdx < contours.size(); feaIdx++)
			{
				feature = contours[feaIdx];
				x0 = feature.x + j;
				y0 = feature.y + i;
				if (!(x0 < gray_image.cols && y0 < gray_image.rows))
				{
					break;
				}
				if (feature.flag)
					similarity += abs(*(m_dxImg + y0 * width + x0) * feature.dx)
					+ abs(*(m_dyImg + y0 * width + x0) * feature.dy);
				else
					similarity += *(m_dxImg + y0 * width + x0) * feature.dx
					+ *(m_dyImg + y0 * width + x0) * feature.dy;

				if (similarity < min_score*feaIdx)
				{
					break;
				}


			}
			similarity /= contours.size();
			if (similarity > score)
			{
				score = similarity;
				result_point.x = j;
				result_point.y = i;
			}
		}
	}



}

Mat draw_contours(Mat im, vector<Feature> contors, Point xy, int width, Scalar color)
{
	Mat result;

	if (im.channels() == 1) {
		vector<Mat> splt;
		for (int i = 0; i < 3; ++i)
			splt.push_back(im.clone());
		merge(splt, result);
	}
	else {
		result = im.clone();
	}
	Point temp_point;
	for (int i = 0; i < contors.size(); i++)
	{
		temp_point.x = contors[i].x + xy.x;
		temp_point.y = contors[i].y + xy.y;
		line(result, temp_point, temp_point, color, width);
	}
	return result;
}

Rect GetBoundingRectangle(vector<Feature> contours)
{
	int minx = numeric_limits<int>::max(), miny = numeric_limits<int>::max();
	int maxx = numeric_limits<int>::min(), maxy = numeric_limits<int>::min();


	for (int i = 0; i < contours.size(); ++i)
	{
		if (minx > contours[i].x)
		{
			minx = contours[i].x;
		}
		else if (maxx < contours[i].x)
		{
			maxx = contours[i].x;
		}

		if (miny > contours[i].y)
		{
			miny = contours[i].y;
		}
		else if (maxy < contours[i].y)
		{
			maxy = contours[i].y;
		}
	}
	return Rect(minx, miny, maxx - minx + 1, maxy - miny + 1);
}

int TresholdOtsu(Mat im)
{
	int height = im.rows;
	int width = im.cols;

	//histogram  
	float histogram[256] = { 0 };
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			histogram[im.at<uchar>(i, j)]++;
		}
	}
	//normalize histogram  
	int size = height * width;
	for (int i = 0; i < 256; i++)
	{
		histogram[i] = histogram[i] / size;
	}

	//average pixel value  
	float avgValue = 0;
	for (int i = 0; i < 256; i++)
	{
		avgValue += i * histogram[i];//整幅图像的平均灰度
	}

	int threshold;
	float maxVariance = 0;
	float w = 0, u = 0;
	for (int i = 0; i < 256; i++)
	{
		w += histogram[i];//假设当前灰度i为阈值, 0~i 灰度的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例      
		u += i * histogram[i];// 灰度i 之前的像素(0~i)的平均灰度值： 前景像素的平均灰度值 

		float t = avgValue * w - u;
		float variance = t * t / (w*(1 - w));
		if (variance > maxVariance)
		{
			maxVariance = variance;
			threshold = i;
		}
	}

	return threshold;
}

void GetImageThreshold(Mat image, int & bestLowThresh, int & bestHighThresh)
{
	int nGrayHistogram[256];
	double por_nGrayHistogram[256];
	double var = 0;
	double maxVar = 0;

	const int GrayLevel = 256;
	double allEpt = 0;
	double Ept[3] = { 0,0,0 };
	double por[3] = { 0,0,0 };
	int lowThresh = 0;
	int highThresh = 0;



	for (int i = 0; i < GrayLevel; i++)
	{
		nGrayHistogram[i] = 0;
	}

	int nPixel;
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			nPixel = image.at<uchar>(i, j);
			nGrayHistogram[nPixel]++;
		}
	}

	int nSum = 0;
	for (int i = 0; i < GrayLevel; i++)
	{
		nSum += nGrayHistogram[i];
	}

	for (int i = 0; i < GrayLevel; i++)
	{
		por_nGrayHistogram[i] = 1.0*nGrayHistogram[i] / nSum;
	}

	for (int i = 0; i < GrayLevel; i++)
	{
		allEpt = i * por_nGrayHistogram[i];
	}

	for (lowThresh = 0; lowThresh < (GrayLevel - 1); lowThresh++)
		for (highThresh = (lowThresh + 1); highThresh < GrayLevel; highThresh++)
		{

			var = 0;
			Ept[0] = Ept[1] = Ept[2] = 0;
			por[0] = por[1] = por[2] = 0;

			for (int i = 0; i < lowThresh; i++)
			{
				por[0] += por_nGrayHistogram[i];
				Ept[0] += i * por_nGrayHistogram[i];
			}
			Ept[0] /= por[0];

			for (int i = lowThresh; i < highThresh; i++)
			{
				por[1] += por_nGrayHistogram[i];
				Ept[1] += i * por_nGrayHistogram[i];
			}
			Ept[1] /= por[1];

			for (int i = highThresh; i < GrayLevel; i++)
			{
				por[2] += por_nGrayHistogram[i];
				Ept[2] += i * por_nGrayHistogram[i];
			}
			Ept[2] /= por[2];

			for (int i = 0; i < 3; i++)
			{
				var += ((Ept[i] - allEpt)*(Ept[i] - allEpt)*por[i]);
			}

			if (var > maxVar)
			{
				maxVar = var;
				bestLowThresh = lowThresh;
				bestHighThresh = highThresh;
			}
		}
}



vector<Mat> cal_search_feature(Mat image, int minGradientMagnitude)
{
	Mat gray_image;
	if (image.channels() == 3)
		cvtColor(image, gray_image, CV_BGR2GRAY);
	else
		gray_image = image.clone();
	int minSqrMagnitude = minGradientMagnitude * minGradientMagnitude;
	Mat dxImage = Mat::zeros(gray_image.size(), CV_64F);
	Mat dyImage = Mat::zeros(gray_image.size(), CV_64F);
	int kernelRadius = 1;
	int x0 = gray_image.cols - 2 * kernelRadius;     //kernelRadius = 1
	int y0 = gray_image.rows - 2 * kernelRadius;
	uchar *frame = gray_image.ptr<uchar>(0);
	double *dxPtr = dxImage.ptr<double>(0);
	double *dyPtr = dyImage.ptr<double>(0);
	int width = gray_image.cols;
#pragma omp parallel for
	for (int y = 0; y < y0; y++)
	{
		int idx0 = (y + kernelRadius)* width + kernelRadius;
		for (int x = 0; x < x0; x++)
		{
			uchar *srcPtr = frame + y * width + x;

			//extend the loop to speed up
			int sumX = *srcPtr - *(srcPtr + 2)
				+ *(srcPtr + width) * 2 - *(srcPtr + width + 2) * 2
				+ *(srcPtr + width * 2) - *(srcPtr + width * 2 + 2);
			int sumY = *srcPtr + *(srcPtr + 1) * 2 + *(srcPtr + 2)
				- *(srcPtr + width * 2) - *(srcPtr + width * 2 + 1) * 2 - *(srcPtr + width * 2 + 2);
			int idx = idx0 + x;
			int grad = sumX * sumX + sumY * sumY;


			if (grad >= minSqrMagnitude)
			{
				double sqrtgrad = sqrt(grad);
				*(dxPtr + idx) = sumX / sqrtgrad;
				*(dyPtr + idx) = sumY / sqrtgrad;
			}
			else
			{
				*(dxPtr + idx) = 0;
				*(dyPtr + idx) = 0;
			}


		}
	}
	vector<Mat> result;
	result.push_back(dxImage);
	result.push_back(dyImage);
	return result;
}
