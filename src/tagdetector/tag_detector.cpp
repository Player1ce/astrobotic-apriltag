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

void astro::sensor_drivers::apriltagdetector::AprilTagDetector::setDrawOutput(bool draw){
  _draw = draw;
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

int astro::sensor_drivers::apriltagdetector::AprilTagDetector::grab(){
  _videoStream >> _raw_img;
  return _processImage();
}

int astro::sensor_drivers::apriltagdetector::AprilTagDetector::_processImage(){
  // Convert image to grey scale
  cv::cvtColor(_raw_img, _grey_img, cv::COLOR_BGR2GRAY);

  // Undistort image
  cv::undistort(_grey_img, _undistort_img, _cameraMatrix, _distortionMatrix);

  vector<AprilTags::TagDetection> detections = _tagDetector->extractTags(_undistort_img);

  if (_draw){
    for (int i = 0; i < detections.size(); i++){
      detections[i].draw(_undistort_img);
    }
    imshow("Test", _undistort_img);
  }
  return detections.size();
}
