/*
Tuan Luong
11/09/2023
*/

#include <tagdetector/tag_detector.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cmath>
/**Test function to convert euler angles to DCM.
 	NOTE: Tag should never have pitch euler(2) near 90 deg. Can result in gimbal lock
	Better way to handle this is quaternion if time permitting*/ 
Eigen::Matrix3d euler2dcm(Eigen::Vector3d & euler) {
  double cphi = cos (euler(0));
  double sphi = sin (euler(0));
  double ctheta = cos (euler(1));
  double stheta = sin (euler(1));
  double cpsi = cos (euler(2));
  double spsi = sin (euler(2));
  Eigen::Matrix3d dcm;
  dcm (0,0) = ctheta * cpsi;
  dcm (0,1) = ctheta * spsi;
  dcm (0,2) = -stheta;
  dcm (1,0) = sphi * stheta * cphi - cphi * spsi;
  dcm (1,1) = sphi * stheta * spsi + cphi * cpsi;
  dcm (1,2) = sphi * ctheta;
  dcm (2,0) = cphi * stheta * cpsi + sphi * spsi;
  dcm (2,1) = cphi * stheta * spsi - sphi * cpsi;
  dcm (2,2) = cphi * ctheta;
  return dcm;
}


 int main() {
	// Known tag position and orientation in NED
	Eigen::Vector3d tag_pos, tag_euler;
	tag_pos << 0.0d, 0.0d, 0.0d;
	tag_euler << 0.0d, 0.0d, 0.0d;
	Eigen::Matrix3d tag_rot_mat = euler2dcm (tag_euler);
	astro::sensor_drivers::apriltagdetector::AprilTagDetector detector;
	detector.setImgSize(1280, 720);
	detector.setInputDev(2);
	cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 396.3014, 0, 629.1302, 0, 396.7859, 362.4086, 0, 0, 1); // fx, 0, px, 0,fy, py, 0, 0, 1
	cv::Mat distortionMatrix = (cv::Mat1d(1, 5) << 0.0079, -0.0256, -0.00008, 0.00008, 0.004); //k1, k2, p1, p2, k3 
	detector.setCameraMatrix(cameraMatrix);
	detector.setDistortionMatrix(distortionMatrix);
	detector.enableViewFinder();
	if (!detector.init()){
		std::cout<<"Init fails\n";
		return 1;
	}
	std::cout<<"detector initialized\n";
	int l = 0;
	while (true){
		auto start = std::chrono::high_resolution_clock::now();
		detector.grab();
		int num_detect = detector.getNumDetection();
		//std::cout << "Num detection: " << num_detect << " | ";
		if (num_detect > 0){
			for (int i = 0; i < num_detect; i++){
				Eigen::Matrix4d T_marker_cam = detector.getRelTransformation(i, 0.166f);
				Eigen::Matrix3d rot_mat;
				Eigen::Vector3d trans_vec;
				detector.getRelCamPose(T_marker_cam, rot_mat, trans_vec);
				
				Eigen::Vector3d test_euler;
				//std::cout << euler_ang * 57.295f << "\n\n";
				//std::cout <<trans_vec<< "\n\n";
				
			}
		}
		else {
			//std::cout << "\n";
		}
		if (cv::waitKey(30) >= 0) break;
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> ms_duration = end - start;
		//std::cout<<ms_duration.count()<<" ms\n";
	}
	float p = l/200.f;
	std::cout<<l<<"\n";
 }
