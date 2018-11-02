#ifndef GLOBALS_HPP
#define GLOBALS_HPP


#include <opencv2/core.hpp>

#define CALIB_PI 3.14159265358979323846
#define CALIB_PI_2 1.57079632679489661923
#define FONT_SIZE 0.4

#define M_PI 3.14159265358979323846

// IMP: The user needs to set some of the variables/pre-processor constants
// prior to running the launch file. If in doubt, see the associated setup
// file for this project.
const std::string workspace_path = "/home/human/drone_tracking/project_code/src/camera_estimation_control/"; 

#define NUM_CAMS 1

// SYSTEM_MODE == 0 means only AprilTag is being used to find camera pose (ie;drone is not in flight).
// SYSTEM_MODE == 1 means only a Rosbag file is being played.
// SYSTEM_MODE == 2 means entire system is running (ie;drone is in flight and PID control is active.).

#define SYSTEM_MODE 2

#if SYSTEM_MODE == 0
	#define THREADSAFE_IMAGE_QUEUE 0
	// For manual or automatic control.
	#define CONTROL 0

#elif SYSTEM_MODE == 1
	#define THREADSAFE_IMAGE_QUEUE 1
	// For manual or automatic control.
	#define CONTROL 0

#else
	#define THREADSAFE_IMAGE_QUEUE 1
	// For manual or automatic control.
	#define CONTROL 1
#endif  



// A few globals for ease - place in common globals file.

//const cv::Size chessBoardDimension(6,9); //width = 6 , height = 9 
//int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
const cv::Scalar RED(0,0,255);
const cv::Scalar GREEN(0,255,0);
const cv::Scalar BLUE(255,0,0);

const cv::Scalar red(0, 0, 255);
const cv::Scalar green(0,255,0);
const cv::Scalar blue(255,0,0);
const cv::Scalar black(0,0,0);
const cv::Scalar yellow(0,255,255);
const cv::Scalar magenta(255,0,255);




// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


#endif
