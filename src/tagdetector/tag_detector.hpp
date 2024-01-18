/**
 * Tuan Luong
 */
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

namespace astro::sensor_drivers::apriltagdetector
{
    class AprilTagDetector {
        private:
            AprilTags::TagDetector* _tagDetector = NULL;
            AprilTags::TagCodes _tagCode = AprilTags::tagCodes25h9;
            vector<AprilTags::TagDetection> _detections;

            bool _draw = false;

            int _width = 800;
            int _height = 600;
            int _devId = 0;

            double _tagSize = 0.168;
            
            cv::Mat _cameraMatrix = (cv::Mat1d(3, 3) << 300, 0, _width/2, 0, 300, _height/2, 0, 0, 1);
            cv::Mat _distortionMatrix = (cv::Mat1d(1, 5) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            double _fx = _cameraMatrix.at<double>(0,0);
            double _fy = _cameraMatrix.at<double>(1,1);
            double _px = _cameraMatrix.at<double>(0,2);
            double _py = _cameraMatrix.at<double>(1,2);

            cv::VideoCapture _videoStream;
            cv::Mat _raw_img;
            cv::Mat _grey_img;
            cv::Mat _undistort_img;

            Eigen::Vector3d _camEuler = Eigen::Vector3d::Zero();
            Eigen::Vector3d _camPos = Eigen::Vector3d::Zero();

            std::vector<double> _x_trans;
            std::vector<double> _y_trans;
            std::vector<double> _z_trans;

            void _processImage ();
            Eigen::Matrix3d _eulertodcm (Eigen::Vector3d & euler);

        public:
            AprilTagDetector();
            ~AprilTagDetector();
            void setCameraMatrix(cv::Mat& cameraMatrix);
            void setDistortionMatrix(cv::Mat& distortionMatrix);
            void setInputDev(int devId);
            void setImgSize(int width, int height);
            void setTagSize(double tagSize);
            void enableViewFinder();
            void disableViewFinder();
            void setTagCode(std::string s);
            bool init();
            void grab();
            
            void getRelCamPose(Eigen::Matrix4d & rel_T, Eigen::Matrix3d & rot_mat, Eigen::Vector3d & trans_vec);
            Eigen::Matrix4d getRelTransformation(int detectionid, float tag_size);
            int getTagId (int detectionid);
            int getNumDetection () {return _detections.size();}

    }  ; 
}