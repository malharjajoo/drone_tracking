#include "Monocamera.hpp"
		
#include <iostream>
#include <cstdio>
#include <fstream>
#include "globals.hpp"


#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/features2d.hpp"

#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


//==================== Helper functions ===================



// 1) Display general info on Matrices
// 2) convert single channel input to multi channel cv::Mat
// 3) convert 3x3 rotation matrix to euler coordinates.
// 4) Calulate known board positions for camera calibration


// Computes P = K * [R|t]
// K : cameraMatrix , 3x3
// R : rotationMatrix , 3x3
// t : translation matrix , 3x1
cv::Mat computeProjectionMatrix(cv::Mat K,cv::Mat R,cv::Mat t)
{
	cv::Mat RTMat(3, 4, CV_64F);
	cv::hconcat(R,t,RTMat); //horizontal concatenation

	return K*RTMat;
}


void printMatrixInfo(cv::Mat M)
{
	cout << "rows = " << M.rows << "\n";
	cout << "cols = " << M.cols << "\n";
	cout << "channels = " << M.channels() << "\n";
}


// cv::Mat.size() gives [ width x height ]
// but we want to display row x col
void findSize(cv::Mat mat)
{
	std::cout << "Size of matrix (row x col )= [" << mat.size().height << " x " << mat.size().width <<"]"  << "\n";
}


// If input matrix dimension is ( number of channels x entries )
// Extract each row and then group each entry into a single entry with 
// multiple channels(= entries in std::vector<cv::Mat> channels vector below) 

// imp thing is usage of cv::merge()
cv::Mat convertToMultiChannel(cv::Mat m)
{
	cv::Mat resMultiChannel;  //Output of function
	
	std::vector<cv::Mat> channels;	

	for(unsigned i = 0 ;i < m.size().height ; i++)
	{
		channels.push_back(m.row(i));
	}
	cv::merge(channels, resMultiChannel);
	return resMultiChannel;
}




void Monocamera::createRegisterer(DroneTracker* droneTracker, Mesh* mesh, Mesh* entireMesh, int camid)
{
    double camera_params[4];
    double distortioncoeffs[5];
    this->getCamparamAndDistortion(camera_params, distortioncoeffs);

    this->registerer = new Registerer(droneTracker,camera_params,distortioncoeffs, mesh, entireMesh, camid);
}


void Monocamera::setCameraWorldPose(okvis::kinematics::Transformation T_WC_in)
{

    this->T_WC.set(T_WC_in.T() );
}

void Monocamera::getCameraWorldPose(okvis::kinematics::Transformation& T)
{
    T.set(this->T_WC.T());
}





bool Monocamera::findCameraToDronePose(cv::Mat img, std::pair<Eigen::Matrix4d,int>& pose_and_ledsfound)
{
    bool gotPose = false;

    gotPose = this->registerer->findPose(img, pose_and_ledsfound, this->displayLevel);

    return gotPose;

}   




/*
//This function computes camera to drone pose and then the drone to world pose
// using information from the April Tag(s).

// Returns a true if -
// 1) Got am image from the camera queue
// 2) A pose was computed by the camera.
// 3) TODO: Maybe add a clause that checks if the camera world pose was initialized or not ?
bool Monocamera::findWorldToDronePose(cv::Mat img, std::pair<Eigen::Matrix4d,int>& pose_and_ledsfound)
{

    bool gotCamPose = false;

    std::pair<Eigen::Matrix4d,int> camerapose_and_ledsfound;
    gotCamPose = this->findCameraToDronePose(img, camerapose_and_ledsfound);

    if(gotCamPose)
    {
        Eigen::Matrix4d T_CS = camerapose_and_ledsfound.first;

        //Eigen::Matrix4d T_WC = this->T_WC.T();
        Eigen::Matrix4d T_WS = this->T_WC.T() * T_CS ;
    

        pose_and_ledsfound.first = T_WS;
        pose_and_ledsfound.second = camerapose_and_ledsfound.second;;
    }

    return gotCamPose;
}
*/




// convertToMultiChannel() converted input matrix to multi channel matrix
// This function converts vector to 2-channel matrix
cv::Mat convertPoint2fTo2ChannelMatrix(std::vector<Point2f> vec)
{
	cv::Mat xes(vec.size(),1,CV_64FC1); // all x-values
	cv::Mat yes(vec.size(),1,CV_64FC1); // all y-values

	for(unsigned i=0; i < vec.size(); ++i)
	{
		xes.at<double>(i,0) = vec[i].x;
		yes.at<double>(i,0) = vec[i].y;
	}
	
	std::vector<cv::Mat> channels;
	channels.push_back(xes);
	channels.push_back(yes);
		
	cv::Mat res2;
	cv::merge(channels,res2);

	return res2;
}




// Converts rotation to Euler angles.
// Please note Euler angles != Rodrigues 
// Input - 3x3 Rotation matrix
// Output - 3x1 Euler Angles ( in degrees )  [ pitch , yaw, roll ]'
void myRot2Euler(const cv::Mat& src, cv::Mat& dst)
{
    if((src.rows == 3) && (src.cols == 3))
    {
        //convert rotaion matrix to 3 angles (pitch, yaw, roll)
        dst = cv::Mat(3, 1, CV_64F);
        double pitch, yaw, roll;

        if(src.at<double>(0,2) < -0.998)
        {
            pitch = -atan2(src.at<double>(1,0), src.at<double>(1,1));
            yaw = -CALIB_PI_2;
            roll = 0.;
        }
        else if(src.at<double>(0,2) > 0.998)
        {
            pitch = atan2(src.at<double>(1,0), src.at<double>(1,1));
            yaw = CALIB_PI_2;
            roll = 0.;
        }
        else
        {
            pitch = atan2(-src.at<double>(1,2), src.at<double>(2,2));
            yaw = asin(src.at<double>(0,2));
            roll = atan2(-src.at<double>(0,1), src.at<double>(0,0));
        }

        // convert to degree
        pitch *= 180./CALIB_PI;
        yaw *= 180./CALIB_PI;
        roll *= 180./CALIB_PI;
       

        dst.at<double>(0,0) = pitch;
        dst.at<double>(1,0) = yaw;
        dst.at<double>(2,0) = roll;
    }
}




bool fexists(const char *filename)
{
  ifstream ifile(filename);
	if(!ifile)
		return false;
	else
		return true;
  
}




//=================================================================================


// constructor 
Monocamera::Monocamera(int id, sensor_msgs::CameraInfoConstPtr cinfo)
:id(id) // not sure about the value
{
	this->pinhole_model = new image_geometry::PinholeCameraModel();
    this->pinhole_model->fromCameraInfo(cinfo);
	
    
	if(!pinhole_model->initialized())	
	{
		ROS_ERROR("Camera model not initialized correctly!");
	}


    this->displayLevel = 0;
}




cv::Mat Monocamera::distortionCoeffs()
{
    return this->pinhole_model->distortionCoeffs();
}







/*

// usage
// string ty =  type2str( M.type() );
//printf("Matrix: %s %dx%d \n", ty.c_str(), M.cols, M.rows );
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

*/
