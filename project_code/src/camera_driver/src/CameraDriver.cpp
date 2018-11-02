/*
This file gets input stream from IP cameras using libVLC library.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> // for converting ROS image to cv::Mat
#include <sstream> // for converting the command line parameter to integer

#include <stdio.h>
#include <stdlib.h>
#include <vlc/vlc.h>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <mutex>
#include <iostream>
#include <cstring>

#include "ros/time.h" // used for filling timestamp in message headers.
#include <camera_info_manager/camera_info_manager.h>

using namespace cv;
using namespace std; 


// define output video resolution  It can either be 1920x1080 or 640x480
#define USE_HD 0


#if USE_HD == 1

	int VIDEO_WIDTH = 1920;
	int VIDEO_HEIGHT = 1080;

#else 

	int VIDEO_WIDTH = 640;
	int VIDEO_HEIGHT = 480;

#endif



// Assumes monocular calibration is already done and results have been stored in a file.
#if USE_HD == 1
    std::string configuration_file1 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/HD/sricam_913777_HD.xml";
    std::string configuration_file2 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/HD/sricam_913778_HD.xml";
    std::string configuration_file3 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/HD/sricam_913734_HD.xml";
#else
    std::string configuration_file1 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/non_HD/sricam_913777.xml";
    std::string configuration_file2 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/non_HD/sricam_913778.xml";
	std::string configuration_file3 = "/home/human/drone_tracking/project_code/src/camera_driver/src/camparams/Intrinsics/non_HD/sricam_913734.xml";
#endif



// Globals
// used for naming ROS topics ....
std::string cam_ns = "/sricam";
std::string cam1str = "/cam_777";
std::string cam2str = "/cam_778";
std::string cam3str = "/cam_734";

std::string transport_type ="/image_raw";



std::string IP_address1="rtsp://192.101.1.101:554/onvif1";
std::string IP_address2="rtsp://192.101.1.102:554/onvif1";
std::string IP_address3="rtsp://192.101.1.103:554/onvif1";



// ROS TF frame ID for 3 cameras.
std::string frame_id_1 = "sricam_777" ;
std::string frame_id_2 = "sricam_778" ;
std::string frame_id_3 = "sricam_734" ;


// Input  - file (in XML format) containing intrinsics of camera.
// Output - Object of type "sensor_msgs::CameraInfo" from the ROS C++ API.
bool getCameraInfo(std::string intrinsics_file, sensor_msgs::CameraInfo& caminfo)
{

	// get file handle 
	cv::FileStorage fs;
	fs.open(intrinsics_file, cv::FileStorage::READ);	

	if(!fs.isOpened())
	{
		ROS_WARN("No input file for publishing camera info.\n");
		ROS_WARN("Please ensure that the path to the camera Intrinsics is correct.");
		return false;
	}



	caminfo.height = VIDEO_HEIGHT;
	caminfo.width = VIDEO_WIDTH;
	caminfo.distortion_model = "plumb_bob"; //"plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).


	// Read the 4 matrices: D,K,R,P and store in output.

	cv::Mat D;
	fs["distortion_coefficients"] >> D;
	//std::cout << "Hey till here 3 \n" ; 
	caminfo.D.resize(5);
	caminfo.D[0] = D.at<double>(0,0);
	caminfo.D[1] = D.at<double>(0,1);
	caminfo.D[2] = D.at<double>(0,2);
	caminfo.D[3] = D.at<double>(0,3);
	caminfo.D[4] = D.at<double>(0,4);


	cv::Mat K;
	cv::Mat R ; 
	fs["camera_matrix"] >> K;
	fs["rectification_matrix"] >> R;

	// this shoudl be o9 elemnts only..
	for(int i = 0; i < K.rows; ++i)
	{
		const double* Mi = K.ptr<double>(i);

		for(int j = 0; j < K.cols; j++)
		{
			caminfo.K[i*K.cols + j] = K.at<double>(i,j);
			caminfo.R[i*K.cols + j] = R.at<double>(i,j);
		}
	}


	cv::Mat P;
	fs["projection_matrix"] >> P;
	for(int i = 0; i < P.rows; ++i)
	{
		const double* Mi = P.ptr<double>(i);

		for(int j = 0; j < P.cols; ++j)
		{
			caminfo.P[i*P.cols + j] = P.at<double>(i,j);
		}
	}

	return true;

}



//================== VLC related code for IP camera ==================


std::mutex imageMutex1;
std::mutex imageMutex2;
std::mutex imageMutex3;


struct ctx 
{ 
    Mat* image;
    uchar* pixels;
};

struct ctx* context1;
struct ctx* context2;
struct ctx* context3;


cv::Mat getFrame1()
{
	return *context1->image;
}

cv::Mat getFrame2()
{
	return *context2->image;
}

cv::Mat getFrame3()
{
	return *context3->image;
}


// Callback prototype to allocate and lock a picture buffer.
// Whenever a new video frame needs to be decoded, the lock callback is invoked.

// Input - data
// Output - p_pixels
void *lock1(void *data, void**p_pixels) 
{
	struct ctx *ctx = (struct ctx*)data;

	// Locking
	imageMutex1.lock();

	// pixel will be stored on image pixel space
	*p_pixels = ctx->pixels; 
	return NULL; 
}




void *lock2(void *data, void**p_pixels) 
{
	struct ctx *ctx = (struct ctx*)data;

	// Locking
	imageMutex2.lock();

	// pixel will be stored on image pixel space
	*p_pixels = ctx->pixels; 
	return NULL; 
}


void *lock3(void *data, void**p_pixels) 
{
	struct ctx *ctx = (struct ctx*)data;

	// Locking
	imageMutex3.lock();

	// pixel will be stored on image pixel space
	*p_pixels = ctx->pixels; 
	return NULL; 
}





// Callback prototype to unlock a picture buffer.
// When the video frame decoding is complete, the unlock callback is invoked.
// This callback might not be needed at all. It is only an indication that the application can now read the pixel values if it needs to.
// More info - https://www.videolan.org/developers/vlc/doc/doxygen/html/group__libvlc__media__player.html#ga024c0f0bba9d3b32a881c40f3cb479bb
void myunlock1(void *data, void *id, void *const *p_pixels)
{ 
	imageMutex1.unlock(); 
} 



void myunlock2(void *data, void *id, void *const *p_pixels)
{ 
	imageMutex2.unlock();
} 

void myunlock3(void *data, void *id, void *const *p_pixels)
{ 
	imageMutex3.unlock();
} 






//==========================================


/* 
	This function publishes IP camera image over ROS topics
	It utilizes image_transport package from ROS.
	Use "rostopic list" in a new bash terminal to see published topics.
	Use "rosrun image_view image_view image:=<name of topic>" to view the actual image separately.
*/


void publish_ipcamera_1(int argc, char** argv, int publish_frequency)
{

	ros::init(argc, argv, "Camera_driver");

	ros::NodeHandle nh1("cam_777");

	image_transport::ImageTransport it1(nh1);



	// Image and Camera info Publishers
	image_transport::Publisher pub1 = it1.advertise(cam_ns + cam1str + transport_type, 1);
	ros::Publisher pub1_caminfo = nh1.advertise<sensor_msgs::CameraInfo>("camera_info", 1);



	// Required for "set_camera_info" service. Convenient (and recommended by ROS) for handling calibration data.
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1(new camera_info_manager::CameraInfoManager(nh1));


	sensor_msgs::ImagePtr msg1;

	ros::Rate loop_rate(publish_frequency);


	//===== Create sensor_msgs::CamInfo Object =======

	sensor_msgs::CameraInfo mycaminfo1;


	bool got1 = getCameraInfo(configuration_file1, mycaminfo1); 


	cinfo1->setCameraInfo(mycaminfo1);


 //=========== Start main publishing loop ==============
         
        
	if( got1 ) 
	{

		const char * const vlc_args[] = 
		{ 
			"-I", "dummy", // Don't use any interface 
			"--ignore-config", // Don't use VLC's config 
			"--extraintf=logger", // Log anything 
			"--verbose=2", // Be much more verbose then normal for debugging purpose 
		};

		libvlc_instance_t *vlcInstance1 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
		
		// Read network stream using RTSP URL ...
		libvlc_media_t *media1 = libvlc_media_new_location(vlcInstance1, IP_address1.c_str()); 


		libvlc_media_player_t *mp1 = libvlc_media_player_new_from_media(media1); 


		libvlc_media_release(media1); 



		context1 = ( struct ctx* )malloc( sizeof( *context1 ) ); 


		//context->imageMutex = CreateMutex(NULL, FALSE, NULL); 
		context1->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context1->pixels = (unsigned char *)context1->image->data;	



		// show blank image
		//imshow("test", *context->image);

		// Key part of the code. for more info - https://www.videolan.org/developers/vlc/doc/doxygen/html/group__libvlc__media__player.html#ga612605f2e5c638d9f4ed59021d714bf0
		libvlc_video_set_callbacks(mp1, lock1, myunlock1, NULL, context1);



		// Set decoded video chroma and dimensions.
 		// only works in combination with libvlc_video_set_callbacks() above.
		// Also, last parameter is pitch = width * BitsPerPixel / 8
		libvlc_video_set_format(mp1, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8); 
		


		//=============================


		int ii = 0;

        cv::Mat mat1;


		while(ros::ok()) 
		{ 
			++ii;

 			// skip few initial frames as they are bad in quality.
			if(ii > 10) 
			{ 
				
				libvlc_media_player_play(mp1);


				//float fps = libvlc_media_player_get_fps(mp1); printf("fps:%f\r\n",fps); 

	            mat1 = getFrame1(); 


				// Check if grabbed frame is actually full with some content
				if(!mat1.empty())
				{

					sensor_msgs::CameraInfo cam_info1 = cinfo1->getCameraInfo();


					// Uses ROS cv_bridge to convert opencv cv::Mat to ROS sensors_msgs::Image type.
					// Then obtain a Ptr to it.
					msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat1).toImageMsg();

					
					msg1->header.frame_id = frame_id_1;

					msg1->header.stamp = ros::Time::now();;

					//caminfo1.header.seq++; // ---> No need to do this since it is done by ROS publisher.
					cam_info1.header.frame_id = frame_id_1;
					cam_info1.header.stamp = msg1->header.stamp;
					


					// publishes raw images and camera calibration information.
					pub1.publish(msg1); 
					pub1_caminfo.publish(cam_info1);


					cv::waitKey(1);
				}
			} 


			ros::spinOnce();
			loop_rate.sleep();
		}

	
		// ======= Tear down of vlc =======

		libvlc_media_player_stop(mp1);
		libvlc_media_player_release(mp1);
		libvlc_release (vlcInstance1);	
		

	}


}

//================ For 2 cameras =========


// The user may wonder why this function is nearly a duplicate of the functoon for single camera...
// Tried to avoid vectorizing so that time is not wasted looping through vector entries while publishing images.
void publish_ipcamera_2(int argc, char** argv, int publish_frequency)
{

	ros::init(argc, argv, "Camera_driver");


	ros::NodeHandle nh1("cam_777");
	ros::NodeHandle nh2("cam_778");

	image_transport::ImageTransport it1(nh1);
	image_transport::ImageTransport it2(nh2);


	// Image and Camera info Publishers
	image_transport::Publisher pub1 = it1.advertise(cam_ns + cam1str + transport_type, 1);
	ros::Publisher pub1_caminfo = nh1.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	image_transport::Publisher pub2 = it2.advertise(cam_ns + cam2str + transport_type, 1);
	ros::Publisher pub2_caminfo = nh2.advertise<sensor_msgs::CameraInfo>("camera_info", 1);


	// Required for "set_camera_info" service. Convenient (and recommended by ROS) for handling calibration data.
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1(new camera_info_manager::CameraInfoManager(nh1));
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo2(new camera_info_manager::CameraInfoManager(nh2));

	sensor_msgs::ImagePtr msg1;
	sensor_msgs::ImagePtr msg2;

	ros::Rate loop_rate(publish_frequency);


	//===== Create sensor_msgs::CamInfo Object =======

	sensor_msgs::CameraInfo mycaminfo1;
	sensor_msgs::CameraInfo mycaminfo2;

	std::vector<bool> gotInfo_vec(3);
	bool got1 = getCameraInfo(configuration_file1, mycaminfo1); 
	bool got2 = getCameraInfo(configuration_file2, mycaminfo2);

	cinfo1->setCameraInfo(mycaminfo1);
	cinfo2->setCameraInfo(mycaminfo2);


 //=========== Start main publishing loop ==============
         
        
	if( got1 && got2 ) 
	{

		const char * const vlc_args[] = 
		{ 
			"-I", "dummy", // Don't use any interface 
			"--ignore-config", // Don't use VLC's config 
			"--extraintf=logger", // Log anything 
			"--verbose=2", // Be much more verbose then normal for debugging purpose 
		};

		libvlc_instance_t *vlcInstance1 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
		libvlc_instance_t *vlcInstance2 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

		// Read network stream using RTSP URL ...
		libvlc_media_t *media1 = libvlc_media_new_location(vlcInstance1, IP_address1.c_str()); 
		libvlc_media_t *media2 = libvlc_media_new_location(vlcInstance2, IP_address2.c_str()); 


		libvlc_media_player_t *mp1 = libvlc_media_player_new_from_media(media1); 
		libvlc_media_player_t *mp2 = libvlc_media_player_new_from_media(media2); 


		libvlc_media_release(media1); 
		libvlc_media_release(media2);


		context1 = ( struct ctx* )malloc( sizeof( *context1 ) ); 
		context2 = ( struct ctx* )malloc( sizeof( *context2 ) );   


		//context->imageMutex = CreateMutex(NULL, FALSE, NULL); 
		context1->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context1->pixels = (unsigned char *)context1->image->data;	

		context2->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context2->pixels = (unsigned char *)context2->image->data;



		// show blank image
		//imshow("test", *context->image);

		// Key part of the code. for more info - https://www.videolan.org/developers/vlc/doc/doxygen/html/group__libvlc__media__player.html#ga612605f2e5c638d9f4ed59021d714bf0
		libvlc_video_set_callbacks(mp1, lock1, myunlock1, NULL, context1);
		libvlc_video_set_callbacks(mp2, lock2, myunlock2, NULL, context2);


		// Set decoded video chroma and dimensions.
 		// only works in combination with libvlc_video_set_callbacks() above.
		// Also, last parameter is pitch = width * BitsPerPixel / 8
		libvlc_video_set_format(mp1, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8); 
		libvlc_video_set_format(mp2, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8);

		


		//=============================


		int ii = 0;

        cv::Mat mat1;
		cv::Mat mat2;


		while(ros::ok()) 
		{ 
			++ii;

 			// skip few initial frames as they are bad in quality.
			if(ii > 10) 
			{ 
				
				libvlc_media_player_play(mp1);
				libvlc_media_player_play(mp2);


				//float fps = libvlc_media_player_get_fps(mp1); printf("fps:%f\r\n",fps); 

	            mat1 = getFrame1(); 
				mat2 = getFrame2();
			


				// Check if grabbed frame is actually full with some content
				if(!mat1.empty() && !mat2.empty())
				{

					sensor_msgs::CameraInfo cam_info1 = cinfo1->getCameraInfo();
					sensor_msgs::CameraInfo cam_info2 = cinfo2->getCameraInfo();



					// Uses ROS cv_bridge to convert opencv cv::Mat to ROS sensors_msgs::Image type.
					// Then obtain a Ptr to it.
					msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat1).toImageMsg();
					msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat2).toImageMsg();

		

					msg1->header.frame_id = frame_id_1;
					msg2->header.frame_id = frame_id_2;


					// For now using the same timestamp for all.
					// MIGHT wish to change this to each having the exact time.
					ros::Time currentTime = ros::Time::now();
					msg1->header.stamp = currentTime;
					msg2->header.stamp = currentTime;


					//caminfo1.header.seq++; // ---> No need to do this since it is done by ROS publisher.
					cam_info1.header.frame_id = frame_id_1;
					cam_info1.header.stamp = currentTime;
					

					cam_info2.header.frame_id = frame_id_2;
					cam_info2.header.stamp = currentTime;
 



					// publishes raw images and camera calibration information.
					pub1.publish(msg1); 
					pub1_caminfo.publish(cam_info1);

					pub2.publish(msg2);
					pub2_caminfo.publish(cam_info2);


					cv::waitKey(1);
				}
			} 


			ros::spinOnce();
			loop_rate.sleep();
		}

	
		// ======= Tear down of vlc =======

		libvlc_media_player_stop(mp1);
		libvlc_media_player_stop(mp2);

		libvlc_media_player_release(mp1);
		libvlc_media_player_release(mp2);


		libvlc_release (vlcInstance1);	
		libvlc_release (vlcInstance2);


	}


}


//====================== For 3 cameras =============


// The user may wonder why this function is nearly a duplicate of the functoon for single camera...
// Tried to avoid vectorizing so that time is not wasted looping through vector entries while publishing images.
void publish_ipcamera_3(int argc, char** argv, int publish_frequency)
{

	ros::init(argc, argv, "Camera_driver");


	ros::NodeHandle nh1("cam_777");
	ros::NodeHandle nh2("cam_778");
	ros::NodeHandle nh3("cam_734");

	image_transport::ImageTransport it1(nh1);
	image_transport::ImageTransport it2(nh2);
	image_transport::ImageTransport it3(nh3);



	// Image and Camera info Publishers
	image_transport::Publisher pub1 = it1.advertise(cam_ns + cam1str + transport_type, 1);
	ros::Publisher pub1_caminfo = nh1.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	image_transport::Publisher pub2 = it2.advertise(cam_ns + cam2str + transport_type, 1);
	ros::Publisher pub2_caminfo = nh2.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	image_transport::Publisher pub3 = it3.advertise(cam_ns + cam3str + transport_type, 1);
	ros::Publisher pub3_caminfo = nh3.advertise<sensor_msgs::CameraInfo>("camera_info", 1);



	// Required for "set_camera_info" service. Convenient (and recommended by ROS) for handling calibration data.
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1(new camera_info_manager::CameraInfoManager(nh1));
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo2(new camera_info_manager::CameraInfoManager(nh2));
	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo3(new camera_info_manager::CameraInfoManager(nh3));


	sensor_msgs::ImagePtr msg1;
	sensor_msgs::ImagePtr msg2;
	sensor_msgs::ImagePtr msg3;

	ros::Rate loop_rate(publish_frequency);


	//===== Create sensor_msgs::CamInfo Object =======

	sensor_msgs::CameraInfo mycaminfo1;
	sensor_msgs::CameraInfo mycaminfo2;
	sensor_msgs::CameraInfo mycaminfo3;

	std::vector<bool> gotInfo_vec(3);
	bool got1 = getCameraInfo(configuration_file1, mycaminfo1); 
	bool got2 = getCameraInfo(configuration_file2, mycaminfo2);
	bool got3 = getCameraInfo(configuration_file3, mycaminfo3);

	cinfo1->setCameraInfo(mycaminfo1);
	cinfo2->setCameraInfo(mycaminfo2);
	cinfo3->setCameraInfo(mycaminfo3);


 //=========== Start main publishing loop ==============
         
        
	if( got1 && got2 && got3 ) 
	{

		const char * const vlc_args[] = 
		{ 
			"-I", "dummy", // Don't use any interface 
			"--ignore-config", // Don't use VLC's config 
			"--extraintf=logger", // Log anything 
			"--verbose=2", // Be much more verbose then normal for debugging purpose 
		};

		libvlc_instance_t *vlcInstance1 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
		libvlc_instance_t *vlcInstance2 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
		libvlc_instance_t *vlcInstance3 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

		// Read network stream using RTSP URL ...
		libvlc_media_t *media1 = libvlc_media_new_location(vlcInstance1, IP_address1.c_str()); 
		libvlc_media_t *media2 = libvlc_media_new_location(vlcInstance2, IP_address2.c_str()); 
		libvlc_media_t *media3 = libvlc_media_new_location(vlcInstance3, IP_address3.c_str()); 


		libvlc_media_player_t *mp1 = libvlc_media_player_new_from_media(media1); 
		libvlc_media_player_t *mp2 = libvlc_media_player_new_from_media(media2); 
		libvlc_media_player_t *mp3 = libvlc_media_player_new_from_media(media3); 


		libvlc_media_release(media1); 
		libvlc_media_release(media2);
		libvlc_media_release(media3);


		context1 = ( struct ctx* )malloc( sizeof( *context1 ) ); 
		context2 = ( struct ctx* )malloc( sizeof( *context2 ) );  
		context3 = ( struct ctx* )malloc( sizeof( *context3 ) );  


		//context->imageMutex = CreateMutex(NULL, FALSE, NULL); 
		context1->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context1->pixels = (unsigned char *)context1->image->data;	

		context2->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context2->pixels = (unsigned char *)context2->image->data;

		context3->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
		context3->pixels = (unsigned char *)context3->image->data;	



		// show blank image
		//imshow("test", *context->image);

		// Key part of the code. for more info - https://www.videolan.org/developers/vlc/doc/doxygen/html/group__libvlc__media__player.html#ga612605f2e5c638d9f4ed59021d714bf0
		libvlc_video_set_callbacks(mp1, lock1, myunlock1, NULL, context1);
		libvlc_video_set_callbacks(mp2, lock2, myunlock2, NULL, context2);
		libvlc_video_set_callbacks(mp3, lock3, myunlock3, NULL, context3);



		// Set decoded video chroma and dimensions.
 		// only works in combination with libvlc_video_set_callbacks() above.
		// Also, last parameter is pitch = width * BitsPerPixel / 8
		libvlc_video_set_format(mp1, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8); 
		libvlc_video_set_format(mp2, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8);
		libvlc_video_set_format(mp3, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8);

		


		//=============================


		int ii = 0;

        cv::Mat mat1;
		cv::Mat mat2;
		cv::Mat mat3;


		while(ros::ok()) 
		{ 
			++ii;

 			// skip few initial frames as they are bad in quality.
			if(ii > 10) 
			{ 
				
				libvlc_media_player_play(mp1);
				libvlc_media_player_play(mp2);
				libvlc_media_player_play(mp3);


				//float fps = libvlc_media_player_get_fps(mp1); printf("fps:%f\r\n",fps); 

	            mat1 = getFrame1(); 
				mat2 = getFrame2();
				mat3 = getFrame3();
			


				// Check if grabbed frame is actually full with some content
				if(!mat1.empty() && !mat2.empty() && !mat3.empty())
				{

					sensor_msgs::CameraInfo cam_info1 = cinfo1->getCameraInfo();
					sensor_msgs::CameraInfo cam_info2 = cinfo2->getCameraInfo();
					sensor_msgs::CameraInfo cam_info3 = cinfo3->getCameraInfo();



					// Uses ROS cv_bridge to convert opencv cv::Mat to ROS sensors_msgs::Image type.
					// Then obtain a Ptr to it.
					msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat1).toImageMsg();
					msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat2).toImageMsg();
					msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat3).toImageMsg();


					

					msg1->header.frame_id = frame_id_1;
					msg2->header.frame_id = frame_id_2;
					msg3->header.frame_id = frame_id_3;


					// For now using the same timestamp for all three.
					// MIGHT wish to change this to each having the exact time.
					ros::Time currentTime = ros::Time::now();
					msg1->header.stamp = currentTime;
					msg2->header.stamp = currentTime;
					msg3->header.stamp = currentTime;


					//caminfo1.header.seq++; // ---> No need to do this since it is done by ROS publisher.
					cam_info1.header.frame_id = frame_id_1;
					cam_info1.header.stamp = currentTime;
					

					cam_info2.header.frame_id = frame_id_2;
					cam_info2.header.stamp = currentTime;
 
					cam_info3.header.frame_id = frame_id_3;
					cam_info3.header.stamp = currentTime;
					


					// publishes raw images and camera calibration information.
					pub1.publish(msg1); 
					pub1_caminfo.publish(cam_info1);

					pub2.publish(msg2);
					pub2_caminfo.publish(cam_info2);

					pub3.publish(msg3);
					pub3_caminfo.publish(cam_info3);

					cv::waitKey(1);
				}
			} 


			ros::spinOnce();
			loop_rate.sleep();
		}

	
		// ======= Tear down of vlc =======

		libvlc_media_player_stop(mp1);
		libvlc_media_player_stop(mp2);
		libvlc_media_player_stop(mp3);

		libvlc_media_player_release(mp1);
		libvlc_media_player_release(mp2);
		libvlc_media_player_release(mp3);

		libvlc_release (vlcInstance1);	
		libvlc_release (vlcInstance2);
		libvlc_release (vlcInstance3);
		

	}


}





int main(int argc, char** argv)
{

	int number_of_cams = atoi(argv[1]);
	int publish_frequency = 30; // max value is 30 FPS.

	// Currently only supporting 3 cameras.
	if(number_of_cams == 1)
	{
		publish_ipcamera_1(argc-1, &argv[1], publish_frequency);
	}

	else if(number_of_cams == 2)
	{
		publish_ipcamera_2(argc-1, &argv[1], publish_frequency);
	}

	else if( number_of_cams == 3)
	{
		publish_ipcamera_3(argc-1, &argv[1], publish_frequency);		
	}

	else
	{
		ROS_ERROR("Too many cameras (passed as command-line argument).\nPass in <=3 cameras.");
	}
}
