# Astrobotic AprilTag 

Simple implementation of AprilTag for initial testing with UA Astrobotic code structure

## Dependencies

OpenCV 

```
sudo apt-get install libopencv-dev
```

Eigen3

```
sudo apt-get install libeigen3-dev
```

## Camera calibration 

### Camera intrinsic matrices

For use with this library, two matrices needs to be obtained for each camera via calibration.

The first matrix is a 3x3 camera matrix which can be defined as 

```
cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, px, 0, fy, px, 0, 0, 1)
```

Where ```fx, fy``` are the focal length in x and y, respectively. ```px, py``` are principle points in x and y, respectively. 

The second matrix is a 1x5 lens distortion matrix which can be defined as 

```
cv::Mat cameraMatrix = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3)
```

Where ```k1, k2, k3``` are the three radial distortion terms. ```p1, p2``` are the two tangential distortion terms.
**Note: Notation for each term might be different for each codebase (opencv vs matlab)**

### Camera calibration methodology

There are multiple methods to obtain these matrices. One tested method is described as follow:

1. Record a video of a checkerboard pattern on a flat surface and convert to image sequence (or capture image sequence directly)
  - The size of the checkered pattern is not important
  - The checkerboard should cover every region in the frame
  - A good video example for best practices is [here](https://www.youtube.com/watch?v=MAoQqhcKKAo&t=7s)
  - For Windows, [AMCAM](https://drive.google.com/drive/folders/10qdEM0TAoTqhkjfphi-SdeKJkJqOG9xX) software is an easy to use app to cconfigure
  - and capture from USB cameras
    
2. Input image sequence into a camera calibration process  (SUGGESTTION: Develop an auto calibration script)
  -  MATLAB Camera Calibrator is a good tool (Need to install appropriate addons)
  -  **Ensure to select correct camera option after importing the images and before calibratin. 3 radial distortion coefficients and 2 tangential distortion coefficients**
  -  After calibration, the quality of the results can be verfified by
      - fx and fy should be close to equal
      - px and py should be roughly half the image size
      - The mean reprojection error is < 1 pixel (Do not remove outliers)
      - When viewing undistorted images on MATLAB, the checkerboard lines are straight
   
## Usage
The detector allows for high level interaction with the original AprilTag C++ code in the class
```astro::sensor_drivers::apriltagdetector::AprilTagDetector```

The class has the following public methods:

```void setCameraMatrix(cv::Mat& cameraMatrix)```  - setter for camera matrix 

```void setDistortionMatrix(cv::Mat& distortionMatrix)``` - setter for camera distortion matrix

```void setInputDev(int devId)``` - setter for the detector input device ID. Each detector can be assign to a camera if multiple is connected

```void setTagCode(std::string tagCode)``` - setter for tag code. Options are "16h5", "25h7", "25h9", "36h9", "36h11"

```void setTagSize(double tagSize)``` - setter for tag size. Use for accurate position of tag relative to camera

```void enableViewFinder()``` - enable view from camera

```void disableViewfinder()``` - disable view from camera

```void setImgSize(int width, int height)``` - setter for input image size. **Ensure camera can achieve that resolution**

```bool init()``` - initialize the tagDetector object. Returns true on success

```void grab()``` - grab and process an image frame

```int getNumDetection() ``` - returns the number of detected tags. This number can be used to setup an iterator to get data from all detections

```int getTagId(int detectionId)``` - returns the ID of specified tag

```Eigen::Vector3d getXYZ(int detectionId)``` - return the XYZ position of specified tag relative to camera. Pos X points out of camera front. Pos Y points to right of camera. Pos Z points down

```double getRange(int detectionId)``` - return the Eucledian distance between camera and specified tag.

```double getBearing(int detectionId)``` - return the bearing of specified tag from camera in radian. Pos is clockwise from the camera optical axis

```double getElevation(int detectionId)``` - return the elevation of specified tag from camera in radian. Pos is up from optical axis.




