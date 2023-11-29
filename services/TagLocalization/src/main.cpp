/*
Tuan Luong
11/09/2023
*/

#include <tagdetector/tag_detector.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

 int main() {
	astro::sensor_drivers::apriltagdetector::AprilTagDetector detector;
	if (!detector.init()){
		std::cout<<"Init fails\n";
		return 1;
	}
	std::cout<<"detector initialized\n";
	detector.setImgSize(960, 720);
	while (true){
		std::cout<<detector.grab()<<"\n";
		if (cv::waitKey(30) >= 0) break;
	}
 }
