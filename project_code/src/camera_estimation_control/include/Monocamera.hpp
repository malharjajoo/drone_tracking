#ifndef MONOCAMERA
#define MONOCAMERA

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/calib3d.hpp>


#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "image_geometry/pinhole_camera_model.h"

#include "Registerer.hpp"
#include "kinematics/Transformation.hpp"
#include "kinematics/operators.hpp"

#include "Mesh.h"

#include <queue>
#include <utility>





class Monocamera
{
	private:
		// Stores calibration information.
		// Additional advantage of using existing classes
		// is you dont need to define stuff like copy constructors or handle
		// deep or shallow copy.
		

		

		int id; // ID used to uniquely identify the camera if multiple are used.

		// decide how much detail to display. 
		// 0 - No display.
		// 1 - Only background subtraction & detection details
		// 2 - Pose estimation details.
		int displayLevel; 

	public:

		// Constructor 
		Monocamera(int id, sensor_msgs::CameraInfoConstPtr cinfo);



		// This is only for rectified image !
		cv::Mat IntrinsicMatrix()
		{
		    
		    double camera_params[4];
		    double distortioncoeffs[5];

		    this->getCamparamAndDistortion(camera_params,distortioncoeffs);

	      	cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		  	K.at<double>(0, 0) = camera_params[0];       //      [ fx   0  cx ]
		  	K.at<double>(1, 1) = camera_params[1];       //      [  0  fy  cy ]
		  	K.at<double>(0, 2) = camera_params[2];       //      [  0   0   1 ]
		  	K.at<double>(1, 2) = camera_params[3];
		  	K.at<double>(2, 2) = 1;

		    return K;
		}




		cv::Mat distortionCoeffs();
		
		
		// Note: Assumes input arrays are of required size (4 and 5).
		void getCamparamAndDistortion(double camera_params[], double distortioncoeffs[])
		{

		    camera_params[0] = this->pinhole_model->fx(); // K.at<double>(0,0); 
		    camera_params[1] = this->pinhole_model->fy(); //K.at<double>(1,1);
		    camera_params[2] = this->pinhole_model->cx(); //K.at<double>(0,2);
		    camera_params[3] = this->pinhole_model->cy(); //K.at<double>(1,2);

		    cv::Mat distMat = this->distortionCoeffs();
		    distortioncoeffs[0] = distMat.at<double>(0);
		    distortioncoeffs[1] = distMat.at<double>(1);
		    distortioncoeffs[2] = distMat.at<double>(2);
		    distortioncoeffs[3] = distMat.at<double>(3);
		    distortioncoeffs[4] = distMat.at<double>(4);

		}


		void createRegisterer(DroneTracker* droneTracker, Mesh* mesh, Mesh* entireMesh, int camid);

		void setCameraWorldPose(okvis::kinematics::Transformation T_WC_in);
		void getCameraWorldPose(okvis::kinematics::Transformation& T);

		//bool findWorldToDronePose(cv::Mat img, std::pair<Eigen::Matrix4d,int>& pose_and_ledsfound);
		bool findCameraToDronePose(cv::Mat img, std::pair<Eigen::Matrix4d,int>& pose_and_ledsfound);

	



		okvis::kinematics::Transformation getWorldPose()
		{
			return this->T_WC;
		}

		int getId() const
		{
			return this->id;
		}

		void setDisplayLevel(int level)
		{
			displayLevel = level;
		}




		// Public Data members
		Registerer* registerer;
		image_geometry::PinholeCameraModel* pinhole_model;
		okvis::kinematics::Transformation T_WC; // world pose of camera
		
};


#endif
