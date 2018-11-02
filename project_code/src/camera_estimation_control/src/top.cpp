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


double maxInput = 0.5; // for manual control of the drone.



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




// Subscribe to 3 images - 2 from stereo cameras and 1 from AR Drone
class MySubscriber 
{
	public:

      cv::Mat teleop_img;


	  MySubscriber(ros::NodeHandle nh, StateEstimator* stateEstimator, controller::Autopilot* autopilot, int total_cams):

	   stateEstimator(stateEstimator),  
       autopilot(autopilot),
	    it_(nh)
	    // TODO: get Undistorted or rectified images ? Because triangulatePoints already 
	    // takes input P1 and P2
	    #if NUM_CAMS == 2
		   	,monocam_1_image_sub( it_, cam1_imageRaw_topic,10 ), 
		   	monocam_2_image_sub( it_,  cam2_imageRaw_topic,10),
	        sync( MySyncPolicy( 10 ), monocam_1_image_sub, monocam_2_image_sub )
       
	   #elif NUM_CAMS == 3
		   	,monocam_1_image_sub( it_,  cam1_imageRaw_topic,10 ), 
		   	 monocam_2_image_sub( it_,  cam2_imageRaw_topic,10 ),
		   	 monocam_3_image_sub( it_,  cam3_imageRaw_topic,10 ),
	         sync( MySyncPolicy( 10 ), monocam_1_image_sub, monocam_2_image_sub, monocam_3_image_sub )
        #endif


	        // If ever wish to use the drone's IMU.
	        //subImu	= nh.subscribe("ardrone/imu", 50, &MySubscriber::imuCallback, this);

	  {

  		#if NUM_CAMS == 1
		// USE it only if testing individual cameras.
		// The subscriber queue size may need to be increased to avoid frame loss.
		// But if my processing is too slow, and I store too many frames, then my algorithm will 
		// end up working on older frames.
			monocam_1_image_sub = it_.subscribe(cam1_imageRaw_topic, 20,&MySubscriber::imagecallback_single_camera, this );
		  	// queue size = 2 (instead of a higher value) because we want to use latest information
		  	// for state estimation.

	  	#elif NUM_CAMS == 2
			sync.registerCallback( boost::bind( &MySubscriber::imagecallback_two_cameras, this, _1, _2 ) );

		// NUM_CAMS == 3
		#elif NUM_CAMS == 3
			sync.registerCallback( boost::bind( &MySubscriber::imagecallback_three_cameras, this, _1, _2, _3 ) );
	  	#endif


	  
		


	  }



	  #if NUM_CAMS == 1
		  void imagecallback_single_camera(const sensor_msgs::ImageConstPtr& img_msg)
		  {
			
				cv::Mat temp1 = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
				cv::Mat img1;

				stateEstimator->cam_list[0]->pinhole_model->rectifyImage(temp1, img1);

				
		        teleop_img = img1.clone();

		       

		        // IMP: Add the image to a queue. 
		        // NOTE: this is just for future extensibility (if adding multithreading,and also more cameras.
		        // in which case it would be good to maintain a queue for each camera.)
		        uint64_t timeStampSeconds = uint64_t(img_msg->header.stamp.sec) ;



		        ImageQueueType item;
				item.img1 = img1;
				item.timestamp = timeStampSeconds;
				stateEstimator->addTimeStampedImage(item); 

		  
	      
	      		#if THREADSAFE_IMAGE_QUEUE == 0  	

					State stateEstimate = stateEstimator->getStateEstimateAndDisplay();
				
					// publish most recent pose to Rviz for visualization ? 
					//stateEstimator->pubPoseRviz();
				#endif
	
		       

		  }



	   #elif NUM_CAMS == 2
		  void imagecallback_two_cameras(const sensor_msgs::ImageConstPtr& img_msg1,const sensor_msgs::ImageConstPtr& img_msg2)
		  {

				cv::Mat temp1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image.clone();
				cv::Mat temp2 = cv_bridge::toCvShare(img_msg2, "bgr8")->image.clone();

				
				// Rectify images 
				cv::Mat img1;
				cv::Mat img2;
				stateEstimator->cam_list[0]->pinhole_model->rectifyImage(temp1, img1);
				stateEstimator->cam_list[1]->pinhole_model->rectifyImage(temp2, img2);
				
				
				teleop_img = img1.clone(); // can choose any..

				
				uint64_t timeStampSeconds = uint64_t(img_msg1->header.stamp.sec) ;


				// Create an item for the Image queue
				ImageQueueType item;
				item.img1 = img1;
				item.img2 = img2;
				item.timestamp = timeStampSeconds;
				
				stateEstimator->addTimeStampedImage(item); 
				
		        	

		        // Call these functions in this subscriber loop only if not using threadsafe image queue.
		        #if THREADSAFE_IMAGE_QUEUE == 0

	          		//State stateEstimate = stateEstimator->getStateEstimate_FillSummary();
	          		//State stateEstimate = stateEstimator->getStateEstimate();
					State stateEstimate = stateEstimator->getStateEstimateAndDisplay();

					 // publish most recent pose to Rviz for visualization ? 
					//stateEstimator->pubPoseRviz();

				#endif
		      
			
		  }

	   
	  #elif NUM_CAMS == 3
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


				teleop_img = img1.clone();


				uint64_t timeStampSeconds = uint64_t(img_msg1->header.stamp.sec) ;


				// add it to queue of each camera.
				ImageQueueType item;
				item.img1 = img1;
				item.img2 = img2;
				item.img3 = img3;
				item.timestamp = timeStampSeconds;
				
				stateEstimator->addTimeStampedImage(item); 

		        
      			#if THREADSAFE_IMAGE_QUEUE == 0
	      			//State stateEstimate = stateEstimator->getStateEstimate_FillSummary();
	      			//State stateEstimate = stateEstimator->getStateEstimate();
					State stateEstimate = stateEstimator->getStateEstimateAndDisplay();
					

			        // publish most recent pose to Rviz for visualization ? 
					stateEstimator->pubPoseRviz();

				#endif

		        
		  }

	 #endif
 
    

	/*
	  // Additional callback for IMU. Not being used  anywhere in current implementation 
	  // but simply given here if ever needed.
	  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
	  {
		    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
		        + msg->header.stamp.nsec / 1000;

		    //Eigen::Vector3d omega_S(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
		    //Eigen::Vector3d acc_S(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);


		    Eigen::Quaterniond q;
		    q.x() = msg->orientation.x;
		    q.y() = msg->orientation.y;
		    q.z() = msg->orientation.z;
		    q.w() = msg->orientation.w;

		    Eigen::Vector3d rpy = radianToDegrees(quaternionToRPY(q));
		    std::cout << "RPY [degress] = (" << rpy(0) << "," << rpy(1) << "," << rpy(2) << ")\n";
	  }
	*/


	private:


      StateEstimator* stateEstimator;
	  controller::Autopilot* autopilot;

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


	  
	  // If subscribing to Drone IMU.
	  //ros::Subscriber subImu ;

};







//========================= Top Level Function ===========================


int main( int argc, char** argv )
{

	
	ros::init(argc,argv, "toplevel_node");
    ros::NodeHandle nh;
    
   	// Connstruct Monocam camera vector.
   	std::vector<Monocamera*> camVector;
   	std::vector<okvis::kinematics::Transformation> cameraWorldPoseVec;


	std::string camera_worldPoseFile = workspace_path + "/src/camera_worldpose.txt";

   	
   	#if NUM_CAMS == 1

    	boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam1_cameraInfo_topic);
		Monocamera* monocam_1 = new Monocamera(0, cinfo_1);
		camVector.push_back(monocam_1);

		std::vector<Eigen::Matrix4d> Tmat_vec = readCameraWorldPosesFromFile(camera_worldPoseFile);         
		okvis::kinematics::Transformation T_WC_0(Tmat_vec[0]);
		cameraWorldPoseVec.push_back(T_WC_0);



	#elif NUM_CAMS == 2
	
		
		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam1_cameraInfo_topic);
		Monocamera* monocam_1 = new Monocamera(0, cinfo_1);
		camVector.push_back(monocam_1);

		boost::shared_ptr< ::sensor_msgs::CameraInfo const> cinfo_2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam2_cameraInfo_topic);
		Monocamera* monocam_2 = new Monocamera(1, cinfo_2);
		camVector.push_back(monocam_2);

		std::vector<Eigen::Matrix4d> Tmat_vec = readCameraWorldPosesFromFile(camera_worldPoseFile);

		okvis::kinematics::Transformation T_WC_0(Tmat_vec[0]);
		cameraWorldPoseVec.push_back(T_WC_0);

		okvis::kinematics::Transformation T_WC_1(Tmat_vec[1]);
		cameraWorldPoseVec.push_back(T_WC_1);


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

		
		std::vector<Eigen::Matrix4d> Tmat_vec = readCameraWorldPosesFromFile(camera_worldPoseFile);

		okvis::kinematics::Transformation T_WC_0(Tmat_vec[0]);
		cameraWorldPoseVec.push_back(T_WC_0);

		okvis::kinematics::Transformation T_WC_1(Tmat_vec[1]);
		cameraWorldPoseVec.push_back(T_WC_1);

		okvis::kinematics::Transformation T_WC_2(Tmat_vec[2]);
		cameraWorldPoseVec.push_back(T_WC_2);



	#endif

	

	// NOTE: The apriltag detector may not be needed.
	// However, keeping future extenssibility in mind, the State
	// Estimator is composed of the Apriltag detector. 

	// Q. Why is the approach below future extensible ?
	// Ans: Because if the cameras were to move/pan around, 
	// the apriltag detector can be invoked by the State Estimator
	// to re-calculate the new pose of the camera(s), as long as the 
	// AprilTag(s) is/are in view of the camera.

	// Construct April Tag detector -
	MyAprilTagDetector* apriltagDetector = new MyAprilTagDetector();
	apriltagDetector->readAprilTagConfigFromFile(workspace_path + "/src/AprilTagConfig.txt");
	

	 // First create controller to initialize drone status information etc, then start image subscriber...
    controller::Autopilot autopilot(nh);
    // TODO: Change this to use relative file paths instead .. for now using full paths because
    // Apparently, ros resolves relative paths from the ~/.ros/ directory so it's not clear what needs to be done ... 
    // See this - https://answers.ros.org/question/11642/write-to-a-file-from-a-c-ros-node/
    autopilot.createWaypointList(workspace_path + "/src/waypoints.txt");



	// Finally provide the stereo camera and april tag detector to the state estimator.
	StateEstimator* stateEstimator = new StateEstimator(camVector, apriltagDetector, &nh, &autopilot);

	// sets a common display level for multiple cameras.
	stateEstimator->setDisplayLevels(0);

	
   


    int total_cams = stateEstimator->getTotalCameras();


    // Set all camera world poses ...
    for(int id = 0; id < cameraWorldPoseVec.size() ; ++id)
    {
    	stateEstimator->setCameraWorldPose_T_WC(id, cameraWorldPoseVec[id]);
    }


   
	MySubscriber my_sub(nh, stateEstimator, &autopilot, total_cams);

	



	//===== Start receiving sensor data and sending commands to/from ardrone topics ======

    // Consider using multi-threaded spinner ? -http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning 
	// this will allow your callbacks to be called from any number of threads.
	// But might mean that we need to make the callback thread safe.. 
	// A way to make it thread safe is to declare everything inside the callback
	// and hence avoid shared variables ...
 

    
    
	#if CONTROL == 1
		 // setup rendering
		SDL_Event event;
		SDL_Init(SDL_INIT_VIDEO);
		SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
		                                     SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
		SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		SDL_RenderClear(renderer);
		SDL_RenderPresent(renderer);
		SDL_Texture * texture;
	#endif



	int i = 0;
	auto start = std::chrono::high_resolution_clock::now();
         

	while(ros::ok())
	{

		// This is done in order to subscribe to topics.
		ros::spinOnce();


	
		#if CONTROL == 1
			SDL_PollEvent(&event);
		    if (event.type == SDL_QUIT) 
		    {
		      break;
		    }

		    float batteryPercent = autopilot.batteryStatus();

			if(batteryPercent<20.0 && batteryPercent!=0.0 )
			{
				//ROS_WARN("Battery LOW: %f \n", batteryPercent) ;
			} 


		    if (!my_sub.teleop_img.empty()) 
			 {

				  // http://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
				  //Convert to SDL_Surface
				  IplImage opencvimg2 = (IplImage) my_sub.teleop_img;
				  IplImage* opencvimg = &opencvimg2;
				  auto frameSurface = SDL_CreateRGBSurfaceFrom(
				      (void*) opencvimg->imageData, opencvimg->width, opencvimg->height,
				      opencvimg->depth * opencvimg->nChannels, opencvimg->widthStep,
				      0xff0000, 0x00ff00, 0x0000ff, 0);
				  if (frameSurface == NULL) 
				  {
				    std::cout << "Couldn't convert Mat to Surface." << std::endl;
				  } 
				  else 
				  {
				    texture = SDL_CreateTextureFromSurface(renderer, frameSurface);
				    SDL_FreeSurface(frameSurface);
				    SDL_RenderClear(renderer);
				    SDL_RenderCopy(renderer, texture, NULL, NULL);
				    SDL_RenderPresent(renderer);
				  }
			}


			
			 //Multiple Key Capture Begins
			const Uint8 *state = SDL_GetKeyboardState(NULL);

			// check states!
	        // TODO: Devise logic so that move() commands (below) are not sent if drone has not taken off or is going to land etc.
			auto droneStatus = autopilot.droneStatus();


	        // manual or automatic control.
	        // Only change the drone controls when user presses the required key. Otherwise continue in current mode.
	        if(state[SDL_SCANCODE_RCTRL])
	        { 
	            autopilot.setAutomatic();
	            std::cout << " [ Set to AUTOMATIC ]" << std::endl;
	        }
	        else if(state[SDL_SCANCODE_SPACE])
	        { 
	            autopilot.setManual();

	            // Clear the state estiamtor queue.
	            std::cout << " [ Set to Manual ]" << std::endl;
	        }


	        //[B]: Display current battery status on terminal
		    if (state[SDL_SCANCODE_B])
		    {
				std::cout<< "Battery: "<<std::to_string(batteryPercent)<<std::endl;
			}


			
			// Manual and automatic control
			if(autopilot.isAutomatic())
			{	
				// Simply print out remaining waypoint list ?
				autopilot.printWaypointList();
			}
			else
			{
			
				// Imp to set them to 0 in every iteration ! Ensures drone does not move when not commanded ...!
				double r = 0;
				double f = 0;
				double l = 0;
				double u = 0;

				
			
				if (state[SDL_SCANCODE_ESCAPE]) 
				{
				  std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus << "\n";
				  bool success = autopilot.estopReset();
				  if(success) {
				    std::cout << " [ OK ]" << std::endl;
				  } else {
				    std::cout << " [FAIL]" << std::endl;
				  }
				}

				if (state[SDL_SCANCODE_T]) 
				{
				  std::cout << "Taking off...                          status=" << droneStatus << "\n";
				  bool success = autopilot.takeoff();
				  if (success) {
				    std::cout << " [ OK ]" << std::endl;
				  } else {
				    std::cout << " [FAIL]" << std::endl;
				  }
				}

				if (state[SDL_SCANCODE_L]) 
				{
				  
				  bool success = autopilot.land();
				  if (success) {
				    std::cout << " [ OK ] \n" ;
				  } else {
				    std::cout << " [FAIL] \n";
				  }
				}
				if (state[SDL_SCANCODE_C]) 
				{
					std::cout << "Requesting flattrim calibration...     status=" << droneStatus << "\n";
					bool success = autopilot.flattrimCalibrate();

					if (success) 
					{
						std::cout << " [ OK ] \n";
					}
					else 
					{
				    	std::cout << " [FAIL] \n";
				 	 }
				}

				
			
				if (state[SDL_SCANCODE_W]) 
				{
				  //std::cout << "Up     status=" << droneStatus << "\n";
				  u = maxInput;
				}

				if (state[SDL_SCANCODE_S]) 
				{
				  //std::cout << "Down     status=" << droneStatus << "\n";
				  // f, l, u, r
				  u = -maxInput;
				}

				if (state[SDL_SCANCODE_A]) 
				{
				  //std::cout << "Yaw Left     status=" << droneStatus << "\n";
				  r = maxInput;
				}

				if (state[SDL_SCANCODE_D]) 
				{
				  //std::cout << "Yaw Right     status=" << droneStatus << "\n";
				  r = -maxInput;
				}

				if (state[SDL_SCANCODE_UP]) 
				{
				  //std::cout << "Forward status=" << droneStatus << "\n";
				  f = maxInput;
				}

				if (state[SDL_SCANCODE_DOWN]) 
				{
				  //std::cout << "Back status=" << droneStatus << "\n";
				  f = -maxInput;
				}  

				if (state[SDL_SCANCODE_LEFT]) 
				{
				  //std::cout << "Left status=" << droneStatus << "\n";
				  l = maxInput;
				}

				if (state[SDL_SCANCODE_RIGHT]) 
				{
				  //std::cout << "Right     status=" << droneStatus << "\n";
				  l = -maxInput;
				}
				

				//printf("\nManual move command = (%f,%f,%f,%f)\n", f,l,u,r); 
				bool success = autopilot.manualMove(f, l, u, r);

				if (success) 
				{
				  //std::cout << " [ OK ]" << std::endl;
				} 
				else 
				{
				  std::cout << " [manualMove FAIL]" << std::endl;
				}



				


			}
		#endif

		++i;
	}
	

	#if CONTROL == 1 
		// make sure to land the drone...
		bool success = autopilot.land();
		
		// cleanup
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
	#endif



	
	delete stateEstimator;

	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	double elapsed_time = elapsed.count();
	std::cout << "\n Elapsed time: " << elapsed_time << " sec\n";
          

	std::cout << "Frequency of control loop (Hz) =" << double(i)/double(elapsed_time) << "\n";

	cv::destroyAllWindows();
	std::cout << "\nFinished execution .....\n";
	return 0;
	

}

