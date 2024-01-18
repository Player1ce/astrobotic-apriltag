#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

// April tags detector and various families that can be selected by command line option
#include "../apriltag/AprilTags/TagDetector.h"
#include "../apriltag/AprilTags/Tag16h5.h"
#include "../apriltag/AprilTags/Tag25h7.h"
#include "../apriltag/AprilTags/Tag25h9.h"
#include "../apriltag/AprilTags/Tag36h9.h"
#include "../apriltag/AprilTags/Tag36h11.h"

#include <cmath>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "tag_detector.hpp"

astro::sensor_drivers::apriltagdetector::AprilTagDetector::AprilTagDetector(){

}

astro::sensor_drivers::apriltagdetector::AprilTagDetector::~AprilTagDetector(){

}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setCameraMatrix(cv::Mat& cameraMatrix){
  _cameraMatrix = cameraMatrix;
  _fx = _cameraMatrix.at<double>(0,0);
  _fy = _cameraMatrix.at<double>(1,1);
  _px = _cameraMatrix.at<double>(0,2);
  _py = _cameraMatrix.at<double>(1,2);
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setDistortionMatrix(cv::Mat& distortionMatrix){
  _distortionMatrix = distortionMatrix;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setInputDev(int devId){
  _devId = devId;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setImgSize(int width, int height){
  _width = width;
  _height = height;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setTagCode(std::string s){
  if (s=="16h5") {
      _tagCode = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      _tagCode = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      _tagCode = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      _tagCode = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      _tagCode = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setTagSize(double tagSize){
  _tagSize = tagSize;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::enableViewFinder(){
  _draw = true;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::disableViewFinder(){
  _draw = false;
}

bool astro::sensor_drivers::apriltagdetector::AprilTagDetector::init(){
  _tagDetector = new AprilTags::TagDetector(_tagCode);

  if (_draw) {
    cv::namedWindow("Test",1);
  }

  _videoStream = cv::VideoCapture(_devId);
  if (!_videoStream.isOpened()){
    std::cout<<"Can't open dev\n";
    return false;
  }
  _videoStream.set(cv::CAP_PROP_FRAME_WIDTH, _width);
  _videoStream.set(cv::CAP_PROP_FRAME_HEIGHT, _height);
  return true;
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::grab(){
  _videoStream >> _raw_img;
  _processImage();
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::_processImage(){
  // Convert image to grey scale
  cv::cvtColor(_raw_img, _grey_img, cv::COLOR_BGR2GRAY);

  // Undistort image
  cv::undistort(_grey_img, _undistort_img, _cameraMatrix, _distortionMatrix);

  _detections = _tagDetector->extractTags(_undistort_img);

  if (_draw){
    for (int i = 0; i < _detections.size(); i++){
      _detections[i].draw(_undistort_img);
    }
    imshow("Test", _undistort_img);
  }
}

int astro::sensor_drivers::apriltagdetector::AprilTagDetector::getTagId(int detectionid){
  return _detections[detectionid].id;
}

Eigen::Matrix4d astro::sensor_drivers::apriltagdetector::AprilTagDetector::getRelTransformation(int detectionid, float tag_size) {
  return _detections[detectionid].getRelativeTransform(tag_size, _fx, _fy, _px, _py);
}

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::getRelCamPose (Eigen::Matrix4d & rel_T, Eigen::Matrix3d & rot_mat, Eigen::Vector3d & trans_vec){
  Eigen::Matrix4d temp = rel_T.inverse();
	Eigen::Matrix3d cv_frame_rot_mat = temp.block(0,0,3,3);

  // Convert to NED frame
	Eigen::Vector3d temp_vec = temp.block(0,3,3,1);
	trans_vec(0) = temp_vec(2);
	trans_vec(1) = -temp_vec(0);
	trans_vec(2) = -temp_vec(1);

	Eigen::Vector3d euler_ang; // In NED frame
	euler_ang(0) = atan (rot_mat(1,0)/ rot_mat(2,2));
	euler_ang(2) = asin (rot_mat(2,0));
	euler_ang(1) = atan(rot_mat(2,1)/ rot_mat(2,2));
  rot_mat = _eulertodcm(euler_ang);
}

Eigen::Matrix3d astro::sensor_drivers::apriltagdetector::AprilTagDetector::_eulertodcm(Eigen::Vector3d & euler) {
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