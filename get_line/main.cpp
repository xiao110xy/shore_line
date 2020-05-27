
// assist_water.cpp: 定义控制台应用程序的入口点。
//

#include "water.h"

#define mask_flag false
int main(int argc, char** argv)
{
	if (argc < 2) {
		//cout << "please input image name\n";
		return 0;
	}
	string image_name(argv[1]);
	Mat image = imread(image_name, IMREAD_COLOR);
	if (!image.data)
	{
		cout << "no image data" << endl;
		return -1;
	}
	// 取出无后缀的文件名及文件所在文件夹路径
	string base_name, base_path;
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
	string assist_image_name(base_path + "assist_" + base_name + string(image_name.end() - 4, image_name.end()));
	string assist_txt_name(base_path + "assist_" + base_name + ".txt");
	string mask_image_name(base_path + "mask_" + base_name + ".png");
	string template_image_name(base_path + "template.png");
	string result_image(base_path + "result_" + base_name + ".jpg");
	string result_txt(base_path + "result_" + base_name + ".txt");
	map<string, string> main_ini;
	main_ini.insert(map<string, string>::value_type("assist_image", assist_image_name));
	main_ini.insert(map<string, string>::value_type("assist_txt", assist_txt_name));
	main_ini.insert(map<string, string>::value_type("mask_image", mask_image_name));
	main_ini.insert(map<string, string>::value_type("template", template_image_name));
	main_ini.insert(map<string, string>::value_type("result_image", result_image));
	main_ini.insert(map<string, string>::value_type("result_txt", result_txt));
	for (int i = 2; i < argc; ++i) {
		string temp(argv[i]);
		string temp1, temp2;
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
	// assit txt
	vector<assist_information> assist_files;
	bool flag = input_assist(image, main_ini, assist_files, true);
	if (!flag) { 
		cout << " assist file error \n";
		return -2;
	}
	//if (mask_flag) {
	//	flag = get_roi(assist_files, main_ini);
	//	if (!flag) {
	//		cout << " mask txt error \n";
	//		return -3;
	//	}
	//}

	//
	//
	compute_water_area(image, assist_files, main_ini["mask_image"]);
	opt_assist_files(assist_files);
	save_file(image, assist_files, main_ini);
	return 1;
}
