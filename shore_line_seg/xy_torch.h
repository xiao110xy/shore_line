


#define torch_1_4


#if defined(_WIN64)
	#include <torch/torch.h>
	#include <torch/script.h>
	#include <memory>
	#include <string>
	#include "opencv2/core/core.hpp"
	#include "opencv2/imgproc/imgproc.hpp"  


class xy_torch {
public:
#if defined(torch_1_0)
	std::shared_ptr<torch::jit::script::Module> module;
#else 
	torch::jit::script::Module module;
#endif

	bool load_model(std::string model_name) {

		std::ifstream fin(model_name);
		if (!fin)
			return false;
		fin.close();
		try {
			torch::DeviceType device_type;
			if (torch::cuda::is_available()) {
				device_type = torch::kCUDA;
			}
			else {
				device_type = torch::kCPU;
			}
			torch::Device device(device_type);
			// Deserialize the ScriptModule from a file using torch::jit::load().
			module = torch::jit::load(model_name, device);
#if defined(torch_1_0)
			if (module == nullptr)
				return false;
#endif
			return true;
		}
		catch (const c10::Error& e) {
			std::cerr << "error loading the model\n";
			return false;
		}
	};
	cv::Mat process_image(cv::Mat image) {

		at::globalContext().setBenchmarkCuDNN(true);
		torch::NoGradGuard no_grad;

#if defined(torch_1_0)
		if (module == nullptr)
			return cv::Mat{};
#else 
		// Deserialize the ScriptModule from a file using torch::jit::load().
		torch::jit::getProfilingMode() = false;
		torch::jit::setGraphExecutorOptimize(false);
#endif


		cv::cvtColor(image, image, CV_BGR2RGB);
		image.convertTo(image, CV_32FC3);

		cv::Mat input_Mat = image.clone();
		cv::Mat channelsConcatenated;
		cv::Mat bgr[3];
		cv::split(input_Mat, bgr);
		float a[3] = { 0.485, 0.456, 0.406 };
		float b[3] = { 0.229, 0.224, 0.225 };
		for (int i = 0; i < 3; ++i) {
			bgr[i] = (bgr[i] / 255.0 - a[i]) / b[i];
		}
		vconcat(bgr[0], bgr[1], channelsConcatenated);
		vconcat(channelsConcatenated, bgr[2], channelsConcatenated);

		std::vector<int64_t> dims{ 1, static_cast<int64_t>(input_Mat.channels()),
								  static_cast<int64_t>(input_Mat.rows),
								  static_cast<int64_t>(input_Mat.cols)
								 };
		//
		torch::DeviceType device_type;
		if (torch::cuda::is_available()) {
			device_type = torch::kCUDA;
			std::cout << "using cuda" << std::endl;
		}
		else {
			device_type = torch::kCPU;
		}
		torch::Device device(device_type);
		//
		//auto options = torch::TensorOptions().dtype(torch::kFloat32).device(device_type);
		auto input = torch::from_blob(channelsConcatenated.data, at::IntArrayRef(dims));
		//input = input.permute({ 0, 3, 1, 2 });
		input = input.to(device);
#if defined(torch_1_0)
		auto output = module->forward({ input }).toTensor();
#else 
		// Deserialize the ScriptModule from a file using torch::jit::load().
		module.eval();
		auto output = module.forward({ input }).toTensor();
#endif


		//output = output.squeeze().detach().permute({ 1, 2, 0 });
		output = output.to(torch::kCPU).to(torch::kFloat32);

		//cv::Mat output_Mat(input_Mat.rows, input_Mat.cols,CV_32FC2);
		//copy the data from out_tensor to resultImg
		//std::memcpy((void *)output_Mat.data, output.data_ptr(), sizeof(torch::kFloat32) * output.numel());
#if defined(torch_1_0)
		cv::Mat output_Mat(cv::Size(input_Mat.cols, 2 * input_Mat.rows), CV_32FC1, output.data<float>());
#else 
		cv::Mat output_Mat(cv::Size(input_Mat.cols, 2 * input_Mat.rows), CV_32FC1, output.data_ptr<float>());
#endif
		std::vector<cv::Mat> results;
		cv::split(output_Mat, results);
		cv::Mat temp_result = cv::Mat::zeros(input_Mat.size(), CV_8UC1) + 50;
		for (int i = 0; i < input_Mat.rows; ++i)
			for (int j = 0; j < input_Mat.cols; ++j)
				if (output_Mat.at<float>(i, j) < output_Mat.at<float>(i + input_Mat.rows, j)) {
					temp_result.at<uchar>(i, j) = 255;
				}
		return temp_result;
	};
};
#endif

