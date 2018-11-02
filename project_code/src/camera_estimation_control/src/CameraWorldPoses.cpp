#include <iostream>

#include <cmath>


#include <iostream>
#include <fstream>

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
#include <opencv2/video/background_segm.hpp>
#include "opencv2/bgsegm.hpp"
#include <opencv2/highgui/highgui.hpp>



#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <controller/Autopilot.hpp>
#include <ardrone_autonomy/Navdata.h>
#include <SDL2/SDL.h>

#include "globals.hpp"
#include "StateEstimator.hpp"


using namespace std;
using namespace okvis;



bool gotAllCameraWorldPoses; // find camera pose for each camera using April Tag.





#if NUM_CAMS == 1

	std::string cam1_imageRaw_topic = "/sricam/cam_777/image_raw";
	std::string cam1_cameraInfo_topic = "/sricam/cam_777/camera_info";

#elif NUM_CAMS == 2

	std::string cam1_imageRaw_topic = "/sricam/cam_777/image_raw";
	std::string cam1_cameraInfo_topic = "/sricam/cam_777/camera_info";

	std::string cam2_imageRaw_topic = "/sricam/cam_778/image_raw";
	std::string cam2_cameraInfo_topic = "/sricam/cam_778/camera_info";

#elif NUM_CAMS == 3

	std::string cam1_imageRaw_topic = "/sricam/cam_777/image_raw";
	std::string cam1_cameraInfo_topic = "/sricam/cam_777/camera_info";

	std::string cam2_imageRaw_topic = "/sricam/cam_778/image_raw";
	std::string cam2_cameraInfo_topic = "/sricam/cam_778/camera_info";

	std::string cam3_imageRaw_topic = "/sricam/cam_734/image_raw";
	std::string cam3_cameraInfo_topic = "/sricam/cam_734/camera_info";
#endif






void findCameraWorldPoseUsingAprilTag(StateEstimator* stateEstimator)
{
	 // Find World pose for each camera.
    if(!gotAllCameraWorldPoses)
    {
    	bool apriltag_detected_in_all_cams = true; // using boolean AND, so will only be true if all are true.


    	 std::vector<cv::Mat> images(stateEstimator->total_cameras);
    

    	 // We pop the image queue. We don't care about the timestamp for AprilTag calibration.
    	#if NUM_CAMS == 1

    	 	ImageQueueType item;
	        bool queue_not_empty = stateEstimator->getTimeStampedImageAndPop(item);

	        if(queue_not_empty)
	        {
	            images[0] = item.img1;
	        }

	    #elif NUM_CAMS == 2

	        ImageQueueType item;
	        bool queue_not_empty = stateEstimator->getTimeStampedImageAndPop(item);

	        if(queue_not_empty)
	        {
	            images[0] = item.img1;
	            images[1] = item.img2;
	        }

	     #elif NUM_CAMS == 3

	        ImageQueueType item;
	        bool queue_not_empty = stateEstimator->getTimeStampedImageAndPop(item);

	        if(queue_not_empty)
	        {
	            images[0] = item.img1;
	            images[1] = item.img2;
	            images[2] = item.img3;
	        }

	    #endif




    	for(int id = 0, total_cams = stateEstimator->getTotalCameras(); id < total_cams; ++id)
    	{
    		bool foundAprilTag = false;

			
			if(queue_not_empty)
			{
				cv::Mat img = images[id];
				foundAprilTag = stateEstimator->findAndSetCameraWorldPose_T_WC(img, id, true);

				if(!foundAprilTag)
	    		{
	    			ROS_INFO("Camera with id = %d can not detect April Tag",id);
	    		}
			}
			else
			{
				ROS_WARN("No images obtained from Camera with id = %d \n",id);
			}
			
    		

    		apriltag_detected_in_all_cams = apriltag_detected_in_all_cams && foundAprilTag;
    	}

		gotAllCameraWorldPoses = apriltag_detected_in_all_cams;

	} 
}




// Subscribe to 3 images - 2 from stereo cameras and 1 from AR Drone
class MySubscriber 
{
	public:



	  MySubscriber(ros::NodeHandle nh, StateEstimator* stateEstimator, int total_cams):

	   stateEstimator(stateEstimator),  
	    it_(nh)
	    // TODO: get Undistorted or rectified images ? Because triangulatePoints already 
	    // takes input P1 and P2
	    #if NUM_CAMS == 2
		   	,monocam_1_image_sub( it_, cam1_imageRaw_topic,2 ), 
		   	monocam_2_image_sub( it_,  cam2_imageRaw_topic,2),
	        sync( MySyncPolicy( 2 ), monocam_1_image_sub, monocam_2_image_sub )
       
	   #elif NUM_CAMS == 3
		   	,monocam_1_image_sub( it_,  cam1_imageRaw_topic,2 ), 
		   	 monocam_2_image_sub( it_,  cam2_imageRaw_topic,2 ),
		   	 monocam_3_image_sub( it_,  cam3_imageRaw_topic,2 ),
	         sync( MySyncPolicy( 2 ), monocam_1_image_sub, monocam_2_image_sub, monocam_3_image_sub )
        #endif

	  {

  		#if NUM_CAMS == 1
		// USE it only if testing individual cameras.
		// The subscriber queue size may need to be increased to avoid frame loss.
		// But if my processing is too slow, and I store too many frames, then my algorithm will 
		// end up working on older frames.
			monocam_1_image_sub = it_.subscribe(cam1_imageRaw_topic, 2,&MySubscriber::imagecallback_single_camera, this );
		  	// queue size = 2 (instead of a higher value) because we want to use latest information
		  	// for state estimation.

	  	#elif NUM_CAMS == 2
			sync.registerCallback( boost::bind( &MySubscriber::imagecallback_two_cameras, this, _1, _2 ) );

		// NUM_CAMS == 3
		#elif NUM_CAMS == 3
			sync.registerCallback( boost::bind( &MySubscriber::imagecallback_three_cameras, this, _1, _2, _3 ) );
	  	#endif


	  
		gotAllCameraWorldPoses = false;


	  }



	  #if NUM_CAMS == 1

		   // this is only used if testing one camera.
		  void imagecallback_single_camera(const sensor_msgs::ImageConstPtr& img_msg1)
		  {

		        

				cv::Mat temp1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image.clone();

				// Rectify images 
				cv::Mat img1;
				stateEstimator->cam_list[0]->pinhole_model->rectifyImage(temp1, img1);

				//cv::Mat img1 = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();



		        // IMP: Add the image to a queue. 
		        // NOTE: this is just for future extensibility (if adding multithreading,and also more cameras.
		        // in which case it would be good to maintain a queue for each camera.)
		        uint64_t timeStampSeconds = uint64_t(img_msg1->header.stamp.sec) ;


		        // add it to camera with id = 0.
		        ImageQueueType item;
				item.img1 = img1;
				item.timestamp = timeStampSeconds;
				stateEstimator->addTimeStampedImage(item); 

		      	findCameraWorldPoseUsingAprilTag(stateEstimator);

		      	// Visualize camera pose in Rviz
		      	if(gotAllCameraWorldPoses)
		      	{
	      			std::cout << "Publishing pose to Rviz ...\n";
			        for(int i = 0; i < 100; ++i)
			        {
			       		 stateEstimator->pubPoseRviz_cameras(0);
			        }
		      	}
		      
		     
		  }



	   #elif NUM_CAMS == 2
		  // Finds drone coordinates by triangulating all points within a bounding rectangle.
		  // Change this to return a single/array of (x,y,z) coordinates
		  void imagecallback_two_cameras(const sensor_msgs::ImageConstPtr& img_msg1,const sensor_msgs::ImageConstPtr& img_msg2)
		  {

				cv::Mat temp1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image.clone();
				cv::Mat temp2 = cv_bridge::toCvShare(img_msg2, "bgr8")->image.clone();

				
				
				// Rectify images 
				cv::Mat img1;
				cv::Mat img2;
				stateEstimator->cam_list[0]->pinhole_model->rectifyImage(temp1, img1);
				stateEstimator->cam_list[1]->pinhole_model->rectifyImage(temp2, img2);

			

				//cv::imshow("Original 1", temp1);
				//cv::imshow("Rectified 1", img1);
				//cv::imshow("Original 2", temp2);
				//cv::imshow("Rectified 2", img2);
				//cv::waitKey(5000);

			
				

				// remove variable later.
				uint64_t timeStampSeconds = uint64_t(img_msg1->header.stamp.sec) ;


				// add it to camera with id = 0 (913_777)
			
				ImageQueueType item;
				item.img1 = img1;
				item.img2 = img2;
				item.timestamp = timeStampSeconds;
				
				stateEstimator->addTimeStampedImage(item); 

		      
		      
				findCameraWorldPoseUsingAprilTag(stateEstimator);


				// Visualize camera pose in Rviz
				if(gotAllCameraWorldPoses)
		      	{
	      		    std::cout << "Publishing pose to Rviz ...\n";
			       for(int i = 0; i < 100; ++i)
			       {
			       		stateEstimator->pubPoseRviz_cameras(0);
					    stateEstimator->pubPoseRviz_cameras(1);
			       }
		      	}
				

		  }

	   
	  #elif NUM_CAMS == 3
		  // Finds drone coordinates by triangulating all points within a bounding rectangle.
		  // Change this to return a single/array of (x,y,z) coordinates
		  void imagecallback_three_cameras(const sensor_msgs::ImageConstPtr& img_msg1,const sensor_msgs::ImageConstPtr& img_msg2, const sensor_msgs::ImageConstPtr& img_msg3)
		  {

		  		
				cv::Mat temp1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image.clone();
				cv::Mat temp2 = cv_bridge::toCvShare(img_msg2, "bgr8")->image.clone();
				cv::Mat temp3 = cv_bridge::toCvShare(img_msg3, "bgr8")->image.clone();

				
				
				// Rectify images 
				cv::Mat img1;
				cv::Mat img2;
				cv::Mat img3;
				stateEstimator->cam_list[0]->pinhole_model->rectifyImage(temp1, img1);
				stateEstimator->cam_list[1]->pinhole_model->rectifyImage(temp2, img2);
				stateEstimator->cam_list[2]->pinhole_model->rectifyImage(temp3, img3);

			
				/*
				cv::imshow("Original 1", temp1);
				cv::imshow("Rectified 1", img1);
				cv::imshow("Original 2", temp2);
				cv::imshow("Rectified 2", img2);
				cv::imshow("Original 3", temp3);
				cv::imshow("Rectified 3", img3);
				cv::waitKey(0);
				*/

				
				// this is not really needed here ... 
				uint64_t timeStampSeconds = uint64_t(img_msg1->header.stamp.sec) ;


				// add it to queue of each camera.
				ImageQueueType item;
				item.img1 = img1;
				item.img2 = img2;
				item.img3 = img3;
				item.timestamp = timeStampSeconds;
				
				stateEstimator->addTimeStampedImage(item); 


		      
		       findCameraWorldPoseUsingAprilTag(stateEstimator);


		       // Visualize camera pose in Rviz

		       if(gotAllCameraWorldPoses)
		      	{
			       std::cout << "Publishing pose to Rviz ...\n";
			       for(int i = 0; i < 100; ++i)
			       {
						stateEstimator->pubPoseRviz_cameras(0);
						stateEstimator->pubPoseRviz_cameras(1);
						stateEstimator->pubPoseRviz_cameras(2);
			       }
		       	}
	        
		  }

	 #endif
 
    

	private:


      StateEstimator* stateEstimator;

	  image_transport::ImageTransport it_;
      
      	#if NUM_CAMS == 1

			image_transport::Subscriber monocam_1_image_sub;

		#elif NUM_CAMS == 2

			//typedef image_transport::SubscriberFilter ImageSubscriber;
			image_transport::SubscriberFilter monocam_1_image_sub;
			image_transport::SubscriberFilter monocam_2_image_sub;

		   typedef message_filters::sync_policies::ExactTime<
		    sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

		  message_filters::Synchronizer< MySyncPolicy > sync;


		#elif NUM_CAMS == 3

			//typedef image_transport::SubscriberFilter ImageSubscriber;
			image_transport::SubscriberFilter monocam_1_image_sub;
			image_transport::SubscriberFilter monocam_2_image_sub;
			image_transport::SubscriberFilter monocam_3_image_sub;

		   typedef message_filters::sync_policies::ExactTime<
		    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image
		  > MySyncPolicy;

		   message_filters::Synchronizer< MySyncPolicy > sync;

	   #endif
	 

};







//========================= Top Level Function ===========================


int main( int argc, char** argv )
{

	
	ros::init(argc,argv, "apriltag_detector");
    ros::NodeHandle nh;
    
   	// Construct Monocam camera vector.
   	std::vector<Monocamera*> camVector;


   	// TODO: Remove the iamge queue. this is not needed since ROS already provides
   	#if NUM_CAMS == 1


    	boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam1_cameraInfo_topic);
		Monocamera* monocam_1 = new Monocamera(0, cinfo_1);
		camVector.push_back(monocam_1);


	#elif NUM_CAMS == 2


		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam1_cameraInfo_topic);
		Monocamera* monocam_1 = new Monocamera(0, cinfo_1);
		camVector.push_back(monocam_1);

		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam2_cameraInfo_topic);
		Monocamera* monocam_2 = new Monocamera(1, cinfo_2);
		camVector.push_back(monocam_2);

	
	#elif NUM_CAMS == 3
	

		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam1_cameraInfo_topic);
		Monocamera* monocam_1 = new Monocamera(0, cinfo_1);
		camVector.push_back(monocam_1);

		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam2_cameraInfo_topic);
		Monocamera* monocam_2 = new Monocamera(1, cinfo_2);
		camVector.push_back(monocam_2);

		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_3 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam3_cameraInfo_topic);
		Monocamera* monocam_3 = new Monocamera(2, cinfo_3);
		camVector.push_back(monocam_3);

	
	#endif

	

	// Construct April Tag detector
	MyAprilTagDetector* apriltagDetector = new MyAprilTagDetector();
	apriltagDetector->readAprilTagConfigFromFile(workspace_path + "/src/AprilTagConfig.txt");
	
	

    
    // Pass in cam vector and apriltag detector to the State Estimator
	StateEstimator* stateEstimator = new StateEstimator(camVector, apriltagDetector, &nh);
	
  

    int total_cams = stateEstimator->getTotalCameras();
	MySubscriber my_sub(nh, stateEstimator, total_cams);

	




    
	// Stop if user presses ctrl+C, or all poses are found
	while(ros::ok() && (!gotAllCameraWorldPoses)) 
	{
		// This is done in order to subscribe to topics.
		ros::spinOnce();
	}



	// Store all the obtained poses into a file.
	std::string filePath =  workspace_path + "src/camera_worldpose.txt" ;
	ofstream _file;
	_file.open(filePath.c_str(), ofstream::out);
	
	if(_file.is_open())
	{
		_file << "# The camera world poses are stored according to the following format: \n";
		_file << "# id \n";
		_file << "# 4x4 Homogenous Transformation Matrix \n\n";


		for(int id; id < total_cams; ++id)
		{
			okvis::kinematics::Transformation temp;
			stateEstimator->getCameraWorldPose_T_WC(id, temp);

			Eigen::Matrix4d camera_T_WC = temp.T();

			_file << id << " " << "\n";

			// Write in this format so that's it easier to convert to a matrix.
			for(int i = 0; i < 4; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					if( !(i==3 && j==3))
					{
						_file  << camera_T_WC(i,j) << " ";
					}
					else
					{
						_file  << camera_T_WC(i,j) << ";";
					}
					


				}

				_file << "\n";
			}
			
			_file << "\n\n";

		}

		_file.flush();
		_file.close();
	}else
	{
		std::cout << "Could not open the file " + filePath << "\n";
	}

				
	

	
	delete stateEstimator;


	cv::destroyAllWindows();
	ROS_INFO("%s","\nFinished.\n Stored Camera(s) world pose in file:" + filePath +".....\n");
	return 0;
	

}

