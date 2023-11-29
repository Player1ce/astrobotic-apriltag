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

            bool _draw = true;

            int _width = 800;
            int _height = 600;
            int _devId = 0;

            double _tagSize = 0.168;
            
            cv::Mat _cameraMatrix = (cv::Mat1d(3, 3) << 300, 0, _width/2, 0, 300, _height/2, 0, 0, 1);
            cv::Mat _distortionMatrix = (cv::Mat1d(1, 5) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            cv::VideoCapture _videoStream;
            cv::Mat _raw_img;
            cv::Mat _grey_img;
            cv::Mat _undistort_img;

            Eigen::Vector3f _camEuler = Eigen::Vector3f::Zero();
            Eigen::Vector3f _camPos = Eigen::Vector3f::Zero();

            int _processImage ();

        public:
            AprilTagDetector();
            ~AprilTagDetector();
            void setCameraMatrix(cv::Mat& cameraMatrix);
            void setDistortionMatrix(cv::Mat& distortionMatrix);
            void setInputDev(int devId);
            void setImgSize(int width, int height);
            void setTagSize(double tagSize);
            void setDrawOutput(bool draw);
            void setTagCode(std::string s);
            bool init();
            int grab();
            // Return range, bearing, elevation
            Eigen::Vector3f getRBE();

    }  ; 
}