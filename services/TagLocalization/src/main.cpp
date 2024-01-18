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
 	NOTE: Euler should never have pitch euler(2) near 90 deg. Can result in gimbal lock
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

void calc_robot_pose (Eigen::Vector3d & tag_pos_world, Eigen::Vector3d & cam_pos_tag, Eigen::Vector3d & cam_pos_rob,
						Eigen::Matrix3d & DCM_tag_world, Eigen::Matrix3d & DCM_cam_tag, Eigen::Matrix3d & DCM_robot_cam,
						Eigen::Vector3d & rob_pos_world, Eigen::Matrix3d & DCM_rob_world) {
	DCM_rob_world = DCM_tag_world * DCM_cam_tag * DCM_robot_cam;
	rob_pos_world = tag_pos_world + DCM_tag_world * cam_pos_tag - DCM_rob_world * cam_pos_rob;
}

void calc_2D_pose (Eigen::Vector3d & robot_pos_world, Eigen::Matrix3d & DCM_rob_world, Eigen::Vector3d & rob_2D_pose){
	rob_2D_pose(0) = robot_pos_world(0);
	rob_2D_pose(1) = robot_pos_world(1);
	rob_2D_pose(2) = atan2(DCM_rob_world(2,1), DCM_rob_world(1,1));
}

int main() {
	// Pose of tag in world and camera in robot frames. Constants
	Eigen::Vector3d tag_pos_world, tag_euler_world, cam_pos_rob, cam_euler_rob;
	tag_pos_world << 0.0d, 0.0d, 0.0d;
	tag_euler_world << 0.0d, 0.0d, 0.0d;
	cam_pos_rob << 0.0d, 0.0d, 0.0d;
	cam_euler_rob << 0.0d, 0.0d, 0.0d;

	// Precompute DCM between tag-world and robot-cam
	Eigen::Matrix3d DCM_tag_world = euler2dcm(tag_euler_world); // This actually calculate DCM from tag to world which is oposite of what we need
	DCM_tag_world.transposeInPlace(); // Tanspose DCM to get correct rotation direction
	Eigen::Matrix3d DCM_robot_cam = euler2dcm(cam_euler_rob);


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
				Eigen::Matrix3d DCM_cam_tag, DCM_rob_world;
				Eigen::Vector3d cam_pos_tag, rob_pos_world, rob_2D_pose;
				detector.getRelCamPose(T_marker_cam, DCM_cam_tag, cam_pos_tag);
				
				// Calculate 3D pose and 2D pose given measurement
				calc_robot_pose(tag_pos_world, cam_pos_tag, cam_pos_rob, DCM_tag_world, DCM_cam_tag, DCM_robot_cam, rob_pos_world, DCM_rob_world);
				calc_2D_pose (rob_pos_world, DCM_rob_world, rob_2D_pose);
				
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
