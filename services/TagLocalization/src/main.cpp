/*
Tuan Luong
11/09/2023
*/

#include <tagdetector/tag_detector.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <chrono>


 int main() {
	astro::sensor_drivers::apriltagdetector::AprilTagDetector detector;
	detector.setImgSize(1280, 720);
	detector.setInputDev(2);
	cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 396.3014, 0, 629.1302, 0, 396.7859, 362.4086, 0, 0, 1); // fx, 0, px, 0,fy, py, 0, 0, 1
	cv::Mat distortionMatrix = (cv::Mat1d(1, 5) << 0.0079, -0.0256, -0.0001, 0.0001, 0.004); //k1, k2, p1, p2, k3 
	detector.setCameraMatrix(cameraMatrix);
	detector.setDistortionMatrix(distortionMatrix);
	//detector.enableViewFinder();
	if (!detector.init()){
		std::cout<<"Init fails\n";
		return 1;
	}
	std::cout<<"detector initialized\n";

	while (true){
		auto start = std::chrono::high_resolution_clock::now();
		detector.grab();
		int num_detect = detector.getNumDetection();
		std::cout << "Num detection: " << num_detect << " | ";
		if (num_detect > 0){
			for (int i = 0; i < num_detect; i++){
				Eigen::Vector3d xyz = detector.getXYZ(i);
				std::cout <<  detector.getRange(i) * 39.37008 << "\n";
			}
		}
		else {
			std::cout << "\n";
		}
		if (cv::waitKey(30) >= 0) break;
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> ms_duration = end - start;
		//std::cout<<ms_duration.count()<<" ms\n";
	}
 }
