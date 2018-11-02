#ifndef STATEESTIMATOR_HPP
#define STATEESTIMATOR_HPP




#include "globals.hpp"
#include "kinematics/Transformation.hpp"
#include "kinematics/operators.hpp"
#include <map>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include <opencv2/core.hpp>


#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#include "globals.hpp"
#include "Utils.h"

#include <geometry_msgs/PoseStamped.h>
#include "Registerer.hpp"
#include "Monocamera.hpp"

#include <controller/Autopilot.hpp>


#include "State.hpp"

#include <utility>





#if THREADSAFE_IMAGE_QUEUE == 1

	#include <thread>
	#include <mutex>
	#include <condition_variable>

#endif 



// Reads file input to obtain each cameras world pose.
std::vector<Eigen::Matrix4d> readCameraWorldPosesFromFile(std::string config_file);

	
// Making a separate class for April Tag since it requires
// camera parameters for obtaining relative Transformation.
// It may be quite inefficient to fetch the camera paramters everytime
// pose needs to be obtained.
class MyAprilTagDetector
{
	private:
		// A map for storing april tag transformations.
		// This is mainly for quicker access if (potentially) required during state estimation
	    std::map<int, okvis::kinematics::Transformation> T_WT_map; 
	    std::map<int, double> tagsize_map;

	public:
		// Constructor
		MyAprilTagDetector()
		{}


		void setTagSizeMap(const std::map<int, double>& tagsize_map)
		{
			this->tagsize_map = tagsize_map;	
		}

		bool getTagPose_T_CT(cv::Mat img, double camera_params[], std::pair<int, okvis::kinematics::Transformation>& myPair , bool displayTags=false);

		// Set transform of tag relative to world frame.	
    	void set_T_WT(int id, okvis::kinematics::Transformation T_WT);
    	void set_T_WT_map(const std::map<int, okvis::kinematics::Transformation>& T_WT_map)
    	{
    		this->T_WT_map = T_WT_map;
    	}

    	void readAprilTagConfigFromFile(std::string config_file);


    	bool get_T_WT(int id, okvis::kinematics::Transformation& output)
    	{
    		if(this->T_WT_map.find(id) != this->T_WT_map.end())
    		{
    			output = this->T_WT_map[id];
    			return true;
    		} 

    		return false;
    	}

    	// ===== Data members ========
    	AprilTags::TagDetector tagDetector = AprilTags::TagDetector(AprilTags::tagCodes36h11); 
    	
};


// Can try a few things 
// 1) Classify a single state estimate as good or bad.
// 2) Store a history of state estimates and check if very sudden movement.

class BadStateRejector
{

	//private: 


	public:


		// Only useful for a very basic check. Does not have any state.
		bool accept(const State& state)
		{

			// default is set to accept. If it fails even a single check it is rejected.
			bool accept = true;


			//1) Some fundamental checks. The drone is never going to be beyond 0.21 radian in 
			// roll or pitch (this was set in the launch file)
			if( std::abs(state.rpy_degrees(0)) > 30 ||
               std::abs(state.rpy_degrees(1)) >= 30 || 

               std::abs(state.r_W(2)) <= 0.07 )
            {
               accept = false;
            }  


			//2) Reject a pose if Rotation matrix is not SO3


			// 3) Reject a pose if it is very different 

            return accept;
		}




};




/*
#if THREADSAFE_STATEESTIMATE_QUEUE == 1


	// FIFO queue.
	// Change to templated version later.
	class StateEstimateQueue
	{
		

		public:
			StateEstimateQueue(int max_size):max_size(max_size)
			{}

			StateEstimateQueue() = default;

			StateEstimateQueue(const ImageQueue& otherQueue) // = delete;
			{
				this->max_size = otherQueue.max_size;
				this->my_queue = otherQueue.my_queue;
			}


			// Blocking push
			void push(const std::pair<State, uint64_t>& item)
			{
				// Lock the queue to prevent another (consumer) thread from removing an entry
				std::unique_lock<std::mutex> mlock(mutex_);	

				if(my_queue.size() >= max_size)
				{
					std::cout << "State Estiamte Queue max size =" << this->max_size << " reached ... \n";
					my_queue.pop(); // remove oldest entry (back of queue)
				}


				my_queue.push(item);


				mlock.unlock();
				cond_.notify_one();		
			} 



			// Blocking pop
			// Waits for push to send a signal
			bool pop(std::pair<State, uint64_t>& item)
			{

				std::unique_lock<std::mutex> mlock(mutex_);


				// avoid undefined behaviour
				// While loop avoids spurious wakes of std::condition
				while( my_queue.empty() )
				{
					cond_.wait(mlock);
					std::cout << "Waiting For State Estimate...\n";	
				}
				

				item = my_queue.front();
				my_queue.pop();

				mlock.unlock();

				return true;
				
			}

			// No need of thread safety for this since there is no modification.
			bool front(std::pair<State, uint64_t>& myPair)
			{
				// avoid undefined behaviour
				if( !my_queue.empty() )
				{
					myPair = my_queue.front();
					return true;
				}
				
				return false;
			}





		private:
			std::queue<std::pair<State, uint64_t>> my_queue; // decide a size ?

			//std::deque<T> my_queue; // LIFO queue


			int max_size;

			std::mutex mutex_;
	 		std::condition_variable cond_;


	};

*/



	// FIFO queue.
	// Change to templated version later.
	class StateEstimateQueue
	{
		private:
			std::queue< std::pair<State, uint64_t> > my_queue; // decide a size ?
			int max_size;

		public:
			StateEstimateQueue(int max_size):max_size(max_size)
			{}

			void push(const std::pair<State,uint64_t>& item)
			{
				
				if(my_queue.size() >= max_size)
				{
					my_queue.pop(); // remove oldest entry (back of queue)
				}

				my_queue.push(item);

				
			} 

			bool pop(std::pair<State, uint64_t>& myPair)
			{

				// avoid undefined behaviour
				if( !my_queue.empty() )
				{
					myPair = my_queue.front();
					my_queue.pop();
					return true;
				}
				

				return false;
			}


			bool front(std::pair<State, uint64_t>& myPair)
			{
				// avoid undefined behaviour
				if( !my_queue.empty() )
				{
					myPair = my_queue.front();
					return true;
				}
				
				return false;
			}


	};






//=========================================


// Add condition on preprocessor on number of cameras.
#if NUM_CAMS == 1

	//typedef std::pair<cv::Mat, uint64_t> ImageQueueType;
	struct image_struct 
	{
		cv::Mat img1;
		uint64_t timestamp;
	};

	typedef image_struct ImageQueueType;
	
#elif NUM_CAMS == 2

	
	struct image_struct 
	{
		cv::Mat img1;
		cv::Mat img2;

		uint64_t timestamp;
	};

	typedef image_struct ImageQueueType;


#elif NUM_CAMS == 3

	struct image_struct 
	{
		cv::Mat img1;
		cv::Mat img2;
		cv::Mat img3;

		uint64_t timestamp;
	};

	typedef image_struct ImageQueueType;


#endif



#if THREADSAFE_IMAGE_QUEUE == 1

		// Thread safe queue.
		// Newest entry at the beginning (FIFO queue)
		class ImageQueue
		{

			public:
				ImageQueue(int max_size):max_size(max_size)
				{}

				ImageQueue()= default;


				ImageQueue(const ImageQueue& otherQueue) // = delete;
				{
					this->max_size = otherQueue.max_size;
					this->my_queue = otherQueue.my_queue;
				}

		  		ImageQueue& operator=(const ImageQueue& otherQueue)
		  		{
		  			this->max_size = otherQueue.max_size;
					this->my_queue = otherQueue.my_queue;

					return *this;
		  		}

				void push(const ImageQueueType& item)
				{
					// Lock the queue to prevent another (consumer) thread from removing an entry
					std::unique_lock<std::mutex> mlock(mutex_);

					
					if(my_queue.size() >= this->max_size)
					{

						std::cout << "Queue max size =" << this->max_size << " reached ... \n";
						// remove oldest entry (back of queue)
						my_queue.pop(); 

					}

					//std::cout << "IamgeQueue size before PUSH=" << my_queue.size() <<" \n";
					// Add latest entry into back of FIFO queue.
					my_queue.push(item);
					//std::cout << "IamgeQueue After PUSH  =" << my_queue.size() << " \n";

					// IMP: we dont need to call mlock.unlock here since it is called
					// when the mlock is destroyed (at the end of this function) anyway.
					// But it is better to call it before the notify signal is sent.
					mlock.unlock();
					cond_.notify_one();

				} 


				bool pop(ImageQueueType& item)
				{

					//  The advantage of creating a unique_lock variable everytime
					// is that unlock() is called even if an exception is thrown.
					// Also. coud have used std::lock_guard() here since we dont require
					// to manually unlock anytime before the function ends ....
					std::unique_lock<std::mutex> mlock(mutex_);


					// While the queue is empty, we "wait" for someone to "notify" us.
					// But we release lock while waiting

					// this while loop is to avoid spurious wakeup calls.
					while( my_queue.empty() )
					{
						cond_.wait(mlock);
						std::cout << "Waiting ...\n";	
					}
					

					
					item = my_queue.front();

					//std::cout << "Queue size before POP=" << my_queue.size() <<" \n";
					my_queue.pop();
					//std::cout << "Queue size after POP=" << my_queue.size() <<" \n";

					// IMP: we dont need to call mlock.unlock here since it is called
					// when the mlock is destroyed (at the end of this function) anyway.

					//TODO: maybe implement a timeout and return false..
					mlock.unlock();
					return true;
				}




			private:
				std::queue<ImageQueueType> my_queue; // decide a size ?

				//std::deque<T> my_queue; // LIFO queue


				int max_size;

				std::mutex mutex_;
		 		std::condition_variable cond_;


		};





#else


		// Normal image queue
		class ImageQueue
		{

			public:
				ImageQueue(int max_size):max_size(max_size)
				{}


				void push(const ImageQueueType& item)
				{


					if(my_queue.size() >= this->max_size)
					{

						std::cout << "Queue max size =" << this->max_size << " reached ... \n";
						// remove oldest entry (back of queue)
						my_queue.pop(); 

					}

			
					// Add latest entry into back of FIFO queue.
					my_queue.push(item);
					

				} 


				bool pop(ImageQueueType& item)
				{

					// While the queue is empty, we "wait" for someone to "notify" us.
					// But we release lock while waiting

					// this while loop is to avoid spurious wakeup calls.
					if( !my_queue.empty() )
					{
						 item = my_queue.front();

						my_queue.pop();
						return true;
					}
					

					return false;
				}



			private:
				std::queue<ImageQueueType> my_queue; // decide a size ?

				int max_size;

		};



#endif













class StateEstimator
{
		
    public:
	    StateEstimator();
	    StateEstimator(std::vector<Monocamera*> camVector, MyAprilTagDetector* apriltagDetector,
	    				 ros::NodeHandle* nh, controller::Autopilot* autopilot);

	    StateEstimator(std::vector<Monocamera*> camVector, MyAprilTagDetector* apriltagDetector,
	    				 ros::NodeHandle* nh);


	    #if THREADSAFE_IMAGE_QUEUE == 1

		    ~StateEstimator()
		    {
		    	std::cout << "\nClosing Image consumer thread now .........\n";
		    	myConsumerThread.join();
		    }

		    void stateEstimationLoop();


		#endif



	    bool getMonocamStateEstimate(cv::Mat img, int id, std::pair<State,int>& state_and_leds);
	 

	 	State getStateEstimate();
	 	// Overload for obtaining a "session" summary.
	 	State getStateEstimate_FillSummary();

	 	
	 	State getStateEstimateAndDisplay();

	    bool findAndSetCameraWorldPose_T_WC(cv::Mat img, int id, bool displayTags=false);
	     // Uses April Tag in horizontal or vertical configuration to get the Camera pose
		bool findCameraPose_T_WC(cv::Mat img, int id, okvis::kinematics::Transformation& T_WC_transform, bool displayTags=false);


	    void getCameraWorldPose_T_WC(int id, okvis::kinematics::Transformation& camera_T_WC);
	    void setCameraWorldPose_T_WC(int id, const okvis::kinematics::Transformation& camera_T_WC);


	  	
	   

	    int findClosestCamera( const std::vector<State>& cameras_state_list, const std::vector<bool> foundPoseVec);

		void load_meshes(); //TODO: remove this later if not using Mesh.
		

		void initializeCameraList(std::vector<Monocamera*> camVector);




		int getTotalCameras() const
		{
			return this->total_cameras;
		}


		// ============= State Queue related ===========


		void addTimeStampedState(State stateEstimate, uint64_t timestampMicroseconds);
		
		bool getTimeStampedStateAndPop(std::pair<State, uint64_t>& myPair);


		// simple overload if timestamp is not required.
		bool getStateDontPop(State& stateEstimate);

		// ============ Image Queue related : push and pop =========

		// A wrapper over each camera image queue
		void addTimeStampedImage(ImageQueueType item)
		{
			image_queue.push(item);			
		}


		bool getTimeStampedImageAndPop(ImageQueueType& item)
		{
			bool queue_not_empty = image_queue.pop(item);
			return queue_not_empty;			
		}




		// ============= Visualization related ==========

		
		// Gets the most recent state estimate and publishes to Rviz.
		void pubPoseRviz();
		void pubPoseRviz_cameras(int id);

		void setDisplayLevels(int displayLevel)
		{
			for(int i = 0; i < this->cam_list.size(); ++i)
			{
				cam_list[i]->setDisplayLevel(displayLevel);
			}

		}






	    // ====== Data memebers =====
	  	// store most recently computed state estimate.
	  	// TODO: Do something with this.
	    State last_stateEstimate;

		std::vector<Monocamera*> cam_list;
		ImageQueue image_queue;


		int total_cameras;
		int total_leds ; // this will be = leds_per_camera * total cameras.
		MyAprilTagDetector* apriltagDetector;

		// TODO: change this to a simple text file later..
		Mesh mesh;  // list of known object point/ LED object coordinates.
		Mesh entireMesh ; // this is full mesh. currently used for visualization but can be used for triangulation
		

		StateEstimateQueue state_queue;


		// publishes pose to Rviz
		ros::Publisher rvizPublisher;
		
		// Make this into a vector maybe ?
		std::vector<ros::Publisher> rviz_cam_publisher_vec;

		ros::NodeHandle* nh;



		// This is for summarizing information of a session.
		SessionSummary summary;


		BadStateRejector badStateRejector;


		#if THREADSAFE_IMAGE_QUEUE == 1

			std::thread myConsumerThread;

		#endif


		controller::Autopilot* autopilot;

};




#endif
