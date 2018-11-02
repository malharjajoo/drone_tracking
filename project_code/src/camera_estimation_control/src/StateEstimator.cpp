#include "StateEstimator.hpp"

#include <sstream>
#include <ros/console.h>
using namespace cv;
using namespace std;
using namespace okvis;

#include <fstream>


#include <map>


//========================= Helper Functions ===========================



// Reads file input to obtain each cameras world pose.
std::vector<Eigen::Matrix4d> readCameraWorldPosesFromFile(std::string config_file)
{

  std::vector<Eigen::Matrix4d> cameraworldpose_vec;
  

  // get file handle 
  std::ifstream myfile (config_file.c_str());

  if (myfile.is_open())
  {
      std::string s;
      std::string delimiter = " ";
      
      // TODO: if multiple April Tags are used then use a map.
      int id;
    
      std::cout << "Reading configuration file for Camera World poses ....\n";

      while ( getline (myfile,s) )
      {
          // If not a comment and it's not a newline.
          if(s[0] != '#' && s.length())
          {
              try
              {
                  // parse string to get April Tag configurations.
                  id = std::stoi(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                  
                  Eigen::Matrix4d T_mat;
                  for(int i = 0 ; i < 4; ++i)
                  {
                      getline (myfile,s);
                      for(int j=0; j < 4; ++j)
                      {
                          T_mat(i,j) = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                      }

                  }

                  cameraworldpose_vec.push_back(T_mat);
              }

              catch(const std::invalid_argument& e)
              {
                  std::cerr << "Invalid argument: " << e.what() << '\n';
                  ROS_ERROR("%s","Please ensure that lines in the camera world pose config file are not incorrectly formatted(stray newlines, or format is incorrect).  \n");
              }
          }


           
      }

  
  }
  else
  {
      ROS_WARN("%s", "Could not find configuration file for camera world pose.");
  }

  return cameraworldpose_vec;

}


void StateEstimator::load_meshes()
{
	// load a mesh given the *.ply file path
   	
    std::string ply_read_path = workspace_path + "mesh_files/led_config.ply";      // known object points.
    std::string ply_read_path_full = workspace_path + "mesh_files/parrot_ar_drone.ply";        // entire object mesh.

    this->mesh.load(ply_read_path);
    this->entireMesh.load(ply_read_path_full);
}


Eigen::Matrix4d getHomogenousMatrixFromState(State state)
{
    Eigen::Matrix4d eigMat;

    Eigen::Matrix3d R = state.q_WS.normalized().toRotationMatrix();

    eigMat.block<3,3>(0,0) = R;
    eigMat(0,3) = state.r_W(0);
    eigMat(1,3) = state.r_W(1);
    eigMat(2,3) = state.r_W(2);

    eigMat(3,0) = 0;
    eigMat(3,1) = 0;
    eigMat(3,2) = 0;
    eigMat(3,3) = 1;

    return eigMat;
}

State getStateFromHomogenousMatrix(const Eigen::Matrix4d& T_WS)
{

    double x,y,z;
    x = T_WS(0,3);
    y = T_WS(1,3);
    z = T_WS(2,3);

    cv::Mat eulers = getRollPitchYawFromHomogenousMatrix(T_WS)  ;

  
    double roll  = radianToDegrees(eulers.at<double>(0));
    double pitch = radianToDegrees(eulers.at<double>(1)) ;
    double yaw = radianToDegrees(eulers.at<double>(2));


    State state(x,y,z,roll,pitch, yaw);
    return state;

}



std::ostream& operator<<(std::ostream& os, const State& stateEstimate)
{
  return os << stateEstimate.r_W(0) << "," << stateEstimate.r_W(1) << "," << stateEstimate.r_W(2) << "," << stateEstimate.rpy_degrees(0) << "," << stateEstimate.rpy_degrees(1) << "," << stateEstimate.rpy_degrees(2) ;
}








//==============================================================








//=================== Related to State Estimation =============

void StateEstimator::initializeCameraList(std::vector<Monocamera*> camVector)
{
    
     //Store all the cameras.
    for(int i = 0; i < camVector.size() ; ++i)
    {
      cam_list.push_back(camVector[i]);
    }

    this->total_cameras = cam_list.size();
}





StateEstimator::StateEstimator(std::vector<Monocamera*> camVector,
                               MyAprilTagDetector* apriltagDetector,
                              ros::NodeHandle* nh)

              : apriltagDetector(apriltagDetector),
               state_queue(1), // This value will depend on the latency of state estiamtor and controller
               
              image_queue(2), // This value will depend on the latency of state estiamtor and controller
               //The number of observations we want to store. 
               // Currently setting this to according to a 30 second flight,
               // Assuming ideal case , total frames = 30 FPS * 30 seconds * 3 cameras = 2700
               // If flight duration increases, this value will need to be increased as well.
               summary(2700), 
               nh(nh)
{

    // By deafault no display.
    this->setDisplayLevels(0);
    initializeCameraList(camVector);

    load_meshes();


    // create the drone tracker and pose registerer FOR ONE CAMERA.
    int leds_per_camera = this->mesh.getNumVertices(); // currently only 4 or 5

    this->total_leds = leds_per_camera * getTotalCameras();



    for(int id = 0; id < this->total_cameras ; ++id)
    {

      // Note that this is done dynamically since the camera params
       // are receieved over ROS topics. This is to ensure flexibility.

        // Also note that we are passing the camera ID to each component so that 
        // display windows can be named uniquely according to unique IDs of the cameras.
        LEDDetector* ledDetector = new LEDDetector(&entireMesh,leds_per_camera, id);
        double camera_params[4];
        double distortioncoeffs[5];
        cam_list[id]->getCamparamAndDistortion(camera_params, distortioncoeffs);

        DroneTracker* droneTracker = new DroneTracker(ledDetector, camera_params, distortioncoeffs, id);
        cam_list[id]->createRegisterer(droneTracker, &mesh, &entireMesh, id);
    }


    // Create pose publisher for rviz
    rvizPublisher = nh->advertise<geometry_msgs::PoseStamped>("/ardrone/worldPose", 1);

    rviz_cam_publisher_vec.resize(this->total_cameras);

    for(int i = 0 ; i < this->total_cameras; ++i)
    {
        rviz_cam_publisher_vec[i] = nh->advertise<geometry_msgs::PoseStamped>("/camera"+SSTR(i)+"/worldPose", 1);
    }
       

    #if THREADSAFE_IMAGE_QUEUE == 1

        myConsumerThread = std::thread(&StateEstimator::stateEstimationLoop,this);

    #endif

}





StateEstimator::StateEstimator(std::vector<Monocamera*> camVector,
                               MyAprilTagDetector* apriltagDetector,
                              ros::NodeHandle* nh,
                              controller::Autopilot* autopilot)

							: apriltagDetector(apriltagDetector),
               state_queue(1), // This value will depend on the latency of state estiamtor and controller
               
              image_queue(2), // This value will depend on the latency of state estiamtor and controller
               //The number of observations we want to store. 
               // Currently setting this to according to a 30 second flight,
               // Assuming ideal case , total frames = 30 FPS * 30 seconds * 3 cameras = 2700
               // If flight duration increases, this value will need to be increased as well.
               summary(2700), 
               nh(nh),
               autopilot(autopilot)
{

    // By deafault no display.
    this->setDisplayLevels(0);
    initializeCameraList(camVector);

   	load_meshes();


    // create the drone tracker and pose registerer FOR ONE CAMERA.
    int leds_per_camera = this->mesh.getNumVertices(); // currently only 4 or 5

    this->total_leds = leds_per_camera * getTotalCameras();



    for(int id = 0; id < this->total_cameras ; ++id)
    {

      // Note that this is done dynamically since the camera params
       // are receieved over ROS topics. This is to ensure flexibility.

        // Also note that we are passing the camera ID to each component so that 
        // display windows can be named uniquely according to unique IDs of the cameras.
        LEDDetector* ledDetector = new LEDDetector(&entireMesh,leds_per_camera, id);
        double camera_params[4];
        double distortioncoeffs[5];
        cam_list[id]->getCamparamAndDistortion(camera_params, distortioncoeffs);

        DroneTracker* droneTracker = new DroneTracker(ledDetector, camera_params, distortioncoeffs, id);
        cam_list[id]->createRegisterer(droneTracker, &mesh, &entireMesh, id);
    }


    // Create pose publisher for rviz
    rvizPublisher = nh->advertise<geometry_msgs::PoseStamped>("/ardrone/worldPose", 1);

    rviz_cam_publisher_vec.resize(this->total_cameras);

    for(int i = 0 ; i < this->total_cameras; ++i)
    {
        rviz_cam_publisher_vec[i] = nh->advertise<geometry_msgs::PoseStamped>("/camera"+SSTR(i)+"/worldPose", 1);
    }
       

    #if THREADSAFE_IMAGE_QUEUE == 1

        myConsumerThread = std::thread(&StateEstimator::stateEstimationLoop,this);

    #endif

}




// TODO: Make this into a weighted average ?
// Can use weights according to the error found with distance .. from Experiments
State computeAverageState(std::vector<State> state_list, const std::vector<bool>& foundPose_vector)
{
    State avg_state;

    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;
    double roll_sum = 0.0;
    double pitch_sum = 0.0;
    double yaw_sum = 0.0;

    int validCount = 0;

    for(int i = 0, len = state_list.size(); i < len; ++i)
    {
        // Average only valid states.
        if(foundPose_vector[i])
        {
          x_sum += state_list[i].r_W(0);
          y_sum += state_list[i].r_W(1);
          z_sum += state_list[i].r_W(2);

          roll_sum += state_list[i].rpy_degrees(0);
          pitch_sum += state_list[i].rpy_degrees(1);
          yaw_sum += state_list[i].rpy_degrees(2);

          ++validCount;
        }
       
    }

    avg_state.setXYZ(x_sum/double(validCount), y_sum/double(validCount),  z_sum/double(validCount) );
    avg_state.setRPY_degrees(roll_sum/double(validCount), pitch_sum/double(validCount), yaw_sum/double(validCount) ); 


    return avg_state;
}




// Finds the index of the closest camera.

// Input: Given we know the world position of the camera (found using April Tags),
// and the world position of the drone.

// Output: Index of camera with least Euclidean distance.
int StateEstimator::findClosestCamera( const std::vector<State>& cameras_state_list, const std::vector<bool> foundPoseVec)
{
    double minDist = INFINITY;
    int index = -1; 

    double dist_sq = 0;

    for(int id=0; id < cameras_state_list.size(); ++id)
    {
        if(foundPoseVec[id])
        {

             okvis::kinematics::Transformation camera_T_WC;
            this->getCameraWorldPose_T_WC(id, camera_T_WC);
            Eigen::Vector3d camera_coordinate = camera_T_WC.r();

            dist_sq = (cameras_state_list[id].r_W(0) - camera_coordinate(0) ) * 
                      (cameras_state_list[id].r_W(0) - camera_coordinate(0) ) + 

                      (cameras_state_list[id].r_W(1) - camera_coordinate(1) ) * 
                      (cameras_state_list[id].r_W(1) - camera_coordinate(1) ) + 

                      (cameras_state_list[id].r_W(2) - camera_coordinate(2) ) * 
                      (cameras_state_list[id].r_W(2) - camera_coordinate(2) ) ;

            if(dist_sq < minDist)
            {
                minDist = dist_sq;
                index = id;
            }

        }
        
    }


    return index;

}





// Returns a boolean whether a state estimate was found or not.
// Also adds the most recent state estimate to the top of the state estimate queue,
State StateEstimator::getStateEstimate()
{

    // Get state estimate from each camera.
    std::vector<State> cameras_state_list(this->total_cameras);
      // Need some way of checking whether a state is valid or not.
    // since we may not receive any state estimate if a camera
    // does not contain any image in image queue or PnP returns false.
    std::vector<bool> foundPose_vector(this->total_cameras);
   

    uint64_t timestamp = -1;
    
    
    float confidence_level = 0.0; // indicates none of the cameras have provided any reading.

    std::vector<cv::Mat> images(this->total_cameras);

    #if NUM_CAMS == 1

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            timestamp = item.timestamp;
        }


    #elif NUM_CAMS == 2

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;

            timestamp = item.timestamp;
        }


    #elif NUM_CAMS==3

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;
            images[2] = item.img3;

            timestamp = item.timestamp;
        }


    #endif



    ////TODO: check why TBB is not giving speedup ?
    for(int i=0 ; i < this->total_cameras; ++i){
    //tbb::parallel_for(0, this->total_cameras,[&](int i){

        if(queue_not_empty)
        {
            cv::Mat img = images[i];

            std::pair<State,int> state_and_leds; // pair containing the state and the LEDs


            bool foundPose = getMonocamStateEstimate(img, i,state_and_leds);

            
            //std::cout << "State Estimation latency =" << latency_vector[i] << "\n";

            // TODO: Decide what needs to be done if no camera provides acceptable measurement.
            // Kalman ?
            bool accept = badStateRejector.accept(state_and_leds.first);

            if(!accept)
            { 
              ROS_WARN("Rejecting pose from camera: %d", i);
            } 

            foundPose_vector[i] = foundPose && accept;

            if(foundPose_vector[i])
            {
                cameras_state_list[i] = state_and_leds.first;
                confidence_level += state_and_leds.second;
            }


        }

        
    } //);


    

    confidence_level = (confidence_level/float(this->total_leds) )*100.0;


    // TODO: Change this to weighted average ?
    State avg_state = computeAverageState(cameras_state_list, foundPose_vector);

    /*
    for(int i=0; i < this->total_cameras; ++i)
    {
        std::cout << "state by camera " << i << ": " << cameras_state_list[i] << "\n";
    } 
    
    std::cout << "avg state  : " << avg_state << "\n";
    */


    // TODO: try the Kalman Filter code here for prediction, 


    // Add final State estimate to queue. 
    this->addTimeStampedState(avg_state, timestamp);
    return avg_state;
    
}






// This is simply an overload of the above. 
// It is separete only for efficiency purposes.
State StateEstimator::getStateEstimate_FillSummary()
{

    // Get state estimate from each camera.
    std::vector<State> cameras_state_list(this->total_cameras);
      // Need some way of checking whether a state is valid or not.
    // since we may not receive any state estimate if a camera
    // does not contain any image in image queue or PnP returns false.
    std::vector<bool> foundPose_vector(this->total_cameras);
    std::vector<int> leds_per_camera(this->total_cameras);
    std::vector<double> latency_vector(this->total_cameras);
    
    uint64_t timestamp = -1;
    
    
    float confidence_level = 0.0; // indicates none of the cameras have provided any reading.

    std::vector<cv::Mat> images(this->total_cameras);


    #if NUM_CAMS == 1

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            timestamp = item.timestamp;
        }


    #elif NUM_CAMS == 2

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;

            timestamp = item.timestamp;
        }


    #elif NUM_CAMS==3

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;
            images[2] = item.img3;

            timestamp = item.timestamp;
        }


    #endif



    ////TODO: check why TBB is not giving speedup ?
    for(int i=0 ; i < this->total_cameras; ++i){
    //tbb::parallel_for(0, this->total_cameras,[&](int i){

        if(queue_not_empty)
        {
            cv::Mat img = images[i];

            std::pair<State,int> state_and_leds; // pair containing the state and the LEDs

            // Find latency of each measurement.
            auto start = std::chrono::high_resolution_clock::now();

            bool foundPose = getMonocamStateEstimate(img, i,state_and_leds);
            

            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            latency_vector[i] = elapsed.count();

            
            //std::cout << "State Estimation latency =" << latency_vector[i] << "\n";

            // TODO: Decide what needs to be done if no camera provides acceptable measurement.
            // Kalman ?
            bool accept = badStateRejector.accept(state_and_leds.first);
            if(!accept){ ROS_WARN("Rejecting pose from camera: %d", i);} 
            foundPose_vector[i] = foundPose && accept;

            if(foundPose_vector[i])
            {
                cameras_state_list[i] = state_and_leds.first;
                leds_per_camera[i] = state_and_leds.second;

                confidence_level += state_and_leds.second;
            }


        }

        
    } //);


    

    confidence_level = (confidence_level/float(this->total_leds) )*100.0;

    DroneAndCameraStatus drone_camera_status(timestamp, cameras_state_list,
        leds_per_camera, foundPose_vector, confidence_level, latency_vector);


    summary.addStatus(drone_camera_status);

    // TODO: Change this to weighted average ?
    State avg_state = computeAverageState(cameras_state_list, foundPose_vector);

    /*
    for(int i=0; i < this->total_cameras; ++i)
    {
        std::cout << "state by camera " << i << ": " << cameras_state_list[i] << "\n";
    } 
    
    std::cout << "avg state  : " << avg_state << "\n";
    */


    // TODO: try the Kalman Filter code here for prediction, 


    // Add final State estimate to queue. 
    this->addTimeStampedState(avg_state, timestamp);
    return avg_state;
    
}




// Returns a boolean whether a state estimate was found or not.
// Also adds the most recent state estimate to the top of the state estimate queue,
State StateEstimator::getStateEstimateAndDisplay()
{

    // Get state estimate from each camera.
    std::vector<State> cameras_state_list(this->total_cameras);
      // Need some way of checking whether a state is valid or not.
    // since we may not receive any state estimate if a camera
    // does not contain any image in image queue or PnP returns false.
    std::vector<bool> foundPose_vector(this->total_cameras);
   
    std::vector<cv::Mat> images(this->total_cameras);

    uint64_t timestamp = -1;


    // Note that all the calls fetching images are blocking pops !
    #if NUM_CAMS == 1

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            timestamp = item.timestamp;
        }


    #elif NUM_CAMS == 2

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;

            timestamp = item.timestamp;
        }


    #elif NUM_CAMS==3

        ImageQueueType item;
        bool queue_not_empty = this->getTimeStampedImageAndPop(item);

        if(queue_not_empty)
        {
            images[0] = item.img1;
            images[1] = item.img2;
            images[2] = item.img3;

            timestamp = item.timestamp;


        }

    #endif
   
     



    //TODO: check why TBB is not giving speedup ?
    for(int i=0 ; i < this->total_cameras; ++i){
    //tbb::parallel_for(0, this->total_cameras,[&](int i){

        if(queue_not_empty)
        {
            
            cv::Mat img = images[i];
            std::pair<State,int> state_and_leds; // pair containing the state and the LEDs

            bool foundPose = getMonocamStateEstimate(img, i, state_and_leds);


            // TODO: Decide what needs to be done if no camera provides acceptabl measurement.
            bool accept = badStateRejector.accept(state_and_leds.first);
            if(!accept){ ROS_WARN("Rejecting pose from camera: %d", i);} 

            foundPose_vector[i] = foundPose && accept;

            if( foundPose_vector[i])
            {
                
                cameras_state_list[i] = state_and_leds.first;


            }

        }
       
    } //);

    
    //======= Verification of World pose by recalculating T_CS = T_CW * T_WS

    
      //int nearestId = findClosestCamera(cameras_state_list,foundPose_vector);
      //std::cout << "Camera nearest to drone = " << nearestId << "\n";


      // TODO: Change this to weighted average ?
      State avg_state =  computeAverageState(cameras_state_list,foundPose_vector);


      for(int i=0; i < this->total_cameras; ++i)
      {
          std::cout << "state by cam[" << i << "]: " << cameras_state_list[i] << "\n";
      } 
      
      std::cout << "avg state  : " << avg_state << "\n\n";
    
     

        // Draw the World to drone pose estimated by each camera, if returned by camera.
     for(int id =0 ; id < this->total_cameras; ++id)
     {
        if(foundPose_vector[id])
        {
             // Draw a mesh going from r_s --> r_w --> r_c .. 
            Eigen::Matrix4d T_WS = getHomogenousMatrixFromState(cameras_state_list[id]);

            Eigen::Matrix4d T_camToDrone = cam_list[id]->getWorldPose().inverse().T() * T_WS;

            /*
            std::cout << "T_WS =" << T_WS << "\n";
            std::cout << "T_WC =" << cam_list[i]->getWorldPose().T() << "\n";
            std::cout << "T_CW" << cam_list[i]->getWorldPose().inverse().T() << "\n";
            std::cout << "T_camTODrone =" << T_camToDrone << "\n\n\n";
            */

            cv::Mat K_matrix = cam_list[id]->IntrinsicMatrix();

            drawObjectMesh(images[id], &entireMesh, T_camToDrone, K_matrix, blue);
            myDrawAxes( images[id], K_matrix, T_camToDrone);

        }
      }
    

      
      // Draw the average mesh for each camera (regardless of whether it found a pose)
     for(int i=0 ; i < this->total_cameras ; ++i)
     {
        
          // Draw a mesh going from r_s --> r_w --> r_c .. this way we are checking T_WS and T_WC.
          Eigen::Matrix4d T_WS = getHomogenousMatrixFromState(avg_state);


          Eigen::Matrix4d T_camToDrone = cam_list[i]->getWorldPose().inverse().T() * T_WS;
          cv::Mat K_matrix = cam_list[i]->IntrinsicMatrix();

          drawObjectMesh(images[i], &entireMesh, T_camToDrone, K_matrix,magenta);
          myDrawAxes( images[i], K_matrix, T_camToDrone);

          cv::namedWindow("Cam"+SSTR(i)+":individual(Blue) estimate",cv::WINDOW_NORMAL);
          cv::imshow("Cam"+SSTR(i)+":individual(Blue) estimate", images[i]); cv::waitKey(1);

      
      }   
        


      
    // TODO: try the Kalman Filter code here for prediction, 

    // Add final State estimate to queue.
    // if at least one camera found an acceptable pose, then fill queue.
    bool atleastOne = false;

    for( int i = 0 ; i < foundPose_vector.size() ; ++i)
    {
        atleastOne = atleastOne || foundPose_vector[i];
    }

    if(atleastOne)
    {
        this->addTimeStampedState(avg_state,timestamp);
    }
    
    return avg_state;
    
}







// This function wraps the pose computed by a single camera into a State Object.
bool StateEstimator::getMonocamStateEstimate(cv::Mat img, int id, std::pair<State,int>& state_and_leds)
{


    // 1) Find camera to drone pose
    std::pair<Eigen::Matrix4d,int> camerapose_and_ledsfound;
    bool gotCamPose = this->cam_list[id]->findCameraToDronePose(img, camerapose_and_ledsfound);

    if(gotCamPose)
    {
        Eigen::Matrix4d T_CS = camerapose_and_ledsfound.first;
        int leds_found = camerapose_and_ledsfound.second;



        //Eigen::Matrix4d T_WC = this->T_WC.T();
        Eigen::Matrix4d T_WS = this->cam_list[id]->T_WC.T() * T_CS ;


        //Fill the output.
        state_and_leds.first = getStateFromHomogenousMatrix(T_WS);
        state_and_leds.second = leds_found;

    }

   /*
   else
   {
      ROS_WARN("[Cam ID:%d] No pose found by this camera, not returning valid state estimate",id);
      ROS_WARN("%s","This may be due to occluded LEDs or poor correspondence...");
   }
    */
   return gotCamPose;
	
}



//  returns true if April Tag was found (and hence camera pose was set successfully.)
bool StateEstimator::findAndSetCameraWorldPose_T_WC(cv::Mat img, int id, bool displayTags)
{
    okvis::kinematics::Transformation camera_T_WC;
    bool foundAprilTag = false;

    foundAprilTag = this->findCameraPose_T_WC(img, id, camera_T_WC, displayTags);

    if(foundAprilTag)
    {
        cam_list[id]->setCameraWorldPose(camera_T_WC);
    }

    return foundAprilTag;

}
    
    
void StateEstimator::getCameraWorldPose_T_WC(int id, okvis::kinematics::Transformation& camera_T_WC)
{
    this->cam_list[id]->getCameraWorldPose(camera_T_WC);
}


void StateEstimator::setCameraWorldPose_T_WC(int id, const okvis::kinematics::Transformation& camera_T_WC)
{
    this->cam_list[id]->setCameraWorldPose(camera_T_WC);
}





bool StateEstimator::findCameraPose_T_WC(cv::Mat img, int id, okvis::kinematics::Transformation& T_WC_transform, bool displayTags/*=false*/)
{

    bool foundAprilTag = false;

    // We know that: T_WC = T_WT * T_TC.
    // 1) Now get T_TC (by inverting T_CT) using April Tag C++ API 
    // fx,fy,cx,cy for a particular camera.
    double camera_params[4];
    double distortioncoeffs[5];

    // We actually don't need the distortion coefficients since the April Tag 
    // library assumes that image is undistorted.
    this->cam_list[id]->getCamparamAndDistortion(camera_params, distortioncoeffs);
        
    // Detect 1 April Tag and get it's id.
    std::pair<int, okvis::kinematics::Transformation> myPair;
    bool found = apriltagDetector->getTagPose_T_CT(img, camera_params, myPair, displayTags);

   

    if(found)
    {
        int foundId = myPair.first;
        okvis::kinematics::Transformation T_CT_transform = myPair.second;


        // We know that: T_WC = T_WT * T_TC.
        // We already have (T_TC)^-1
        // 2) Now get T_WT
        okvis::kinematics::Transformation T_WT_transform;
        bool found = this->apriltagDetector->get_T_WT(foundId, T_WT_transform);


        // If the ID for the ones that you read from the file matches the current one detected
        if(found)
        {
            Eigen::Matrix4d T_WT = T_WT_transform.T();

            
            //std::cout << "\n World to AprilTag pose T_WT = \n" << T_WT << "\n";

            //std::cout << "\n Camera to AprilTag pose T_CT  = \n" << T_CT_transform.T() << "\n";
            Eigen::Matrix4d T_TC = T_CT_transform.inverse().T();


            Eigen::Matrix4d T_WC = T_WT * T_TC;
            //std::cout << "\n World to camera pose T_WC = \n" << T_WC << "\n";

            cv::Mat temp = getRollPitchYawFromHomogenousMatrix(T_WC);
            double roll = radianToDegrees(temp.at<double>(0));
            double pitch = radianToDegrees(temp.at<double>(1));
            double yaw = radianToDegrees(temp.at<double>(2));

            ROS_INFO("[World to camera]: \n x,y,z (in metres) = (%f,%f,%f) \n RPY (in degrees) = (%f,%f,%f) \n", T_WC(0,3), T_WC(1,3), T_WC(2,3), roll, pitch, yaw);

            T_WC_transform.set(T_WC);

            foundAprilTag = true;
        }
        else
        {
            ROS_WARN("%s", "Could not find April Tag world poses...\n");
            foundAprilTag = false;
        }

  
    }
   
    return foundAprilTag;

}



// publish world pose of cameras to Rviz
void StateEstimator::pubPoseRviz_cameras(int id)
{
    
    okvis::kinematics::Transformation T_WC;
    this->cam_list[id]->getCameraWorldPose(T_WC);


    // Extract position and quaternion 
    geometry_msgs::PoseStamped myPose;

    // parent frame is world frame.
    myPose.header.frame_id = "world";
    // rviz can't handle original timestamps. won't matter, it's visualisation only...
    myPose.header.stamp = ros::Time::now();


    // fill position
    Eigen::Vector3d r_W = T_WC.r();
    myPose.pose.position.x = r_W[0];
    myPose.pose.position.y = r_W[1];
    myPose.pose.position.z = r_W[2]; 

    // fill orientation
    Eigen::Quaterniond q_WS = T_WC.q();

    myPose.pose.orientation.x = q_WS.x();
    myPose.pose.orientation.y = q_WS.y();
    myPose.pose.orientation.z = q_WS.z();
    myPose.pose.orientation.w = q_WS.w();

    
    
    rviz_cam_publisher_vec[id].publish(myPose);
}




// Gets the most recent state estimate from a queue and publishes to Rviz.
void StateEstimator::pubPoseRviz()
{
    // get most recent state estimate.
    State stateEstimate;

    // Imp, should not pop state queue.
    bool queue_not_empty = this->getStateDontPop(stateEstimate);


    // Extract position and quaternion 
    if(queue_not_empty)
    {
        geometry_msgs::PoseStamped myPose;

        // parent frame is world frame.
        myPose.header.frame_id = "world";
        // rviz can't handle original timestamps. won't matter, it's visualisation only...
        myPose.header.stamp = ros::Time::now();


        // fill position
        myPose.pose.position.x = stateEstimate.r_W[0];
        myPose.pose.position.y = stateEstimate.r_W[1];
        myPose.pose.position.z = stateEstimate.r_W[2]; 

        // fill orientation
        myPose.pose.orientation.x = stateEstimate.q_WS.x();
        myPose.pose.orientation.y = stateEstimate.q_WS.y();
        myPose.pose.orientation.z = stateEstimate.q_WS.z();
        myPose.pose.orientation.w = stateEstimate.q_WS.w();

        rvizPublisher.publish(myPose);
    }

    
}



#if THREADSAFE_IMAGE_QUEUE == 1

    void StateEstimator::stateEstimationLoop()
    {

        // Maybe change this to stop when image queue (of all cameras) is empty ?
        while(ros::ok()) 
        {
            //auto start = std::chrono::high_resolution_clock::now(); 
          
            this->getStateEstimateAndDisplay();
            //State stateEstimate = this->getStateEstimate_FillSummary();
            //State stateEstimate = this->getStateEstimate();
           
            //auto finish = std::chrono::high_resolution_clock::now();
            //std::chrono::duration<double> elapsed = finish - start;
            //std::cout << "\nTime taken for state estimation: " << elapsed.count() << " sec\n";
            


            // publish most recent pose to Rviz for visualization ? 
            //stateEstimator->pubPoseRviz();

            #if CONTROL == 1
              // TODO: remove this later when multi-threading is introduced.
              if(autopilot->isAutomatic())
              {
                  std::cout << "[ AUTOPILOT is automatic now ] ..\n";

                  std::pair<State,uint64_t> myPair;
                  if(getTimeStampedStateAndPop(myPair))
                  {
                    uint64_t timestampMicroseconds = myPair.second *1000000;
                    autopilot->pidControl(timestampMicroseconds, myPair.first);
                  }
                  //else{autopilot->hover();}
                 
              } 
            #endif
            

          //cv::destroyAllWindows();
              
      }
  }

#endif




// ============= State Queue related ===========


void StateEstimator::addTimeStampedState(State stateEstimate, uint64_t timestampMicroseconds)
{
  state_queue.push(std::make_pair(stateEstimate, timestampMicroseconds));
}


bool StateEstimator::getTimeStampedStateAndPop(std::pair<State, uint64_t>& myPair)
{
  bool queue_not_empty = state_queue.pop(myPair);
  return queue_not_empty;
}

  // simple overload if timestamp is not required.
bool StateEstimator::getStateDontPop(State& stateEstimate)
{
  std::pair<State, uint64_t> myPair;
  bool queue_not_empty = state_queue.front(myPair);

  if(queue_not_empty)
  {
      stateEstimate = myPair.first;
  }

  return queue_not_empty;
}




//=============== April Tag detection functions ===========


// Map is used since there may be several April Tags
void MyAprilTagDetector::set_T_WT(int id, kinematics::Transformation T_WT)
{
	this->T_WT_map[id] = T_WT;
}



// Reads file input to obtain April Tag configuration.
void MyAprilTagDetector::readAprilTagConfigFromFile(std::string config_file)
{


  std::map<int, okvis::kinematics::Transformation> apriltagpose_map;
  std::map<int, double> tagsize_map;
  

  // get file handle 
  std::ifstream myfile (config_file.c_str());

  if (myfile.is_open())
  {
      std::string s;
      std::string delimiter = " ";
      
      // TODO: if multiple April Tags are used then use a map.
      int id;
      double tag_size;

      std::cout << "Reading configuration file for April Tag ....\n";

      while ( getline (myfile,s) )
      {
          // If not a comment and it's not a newline.
          if(s[0] != '#' && s.length())
          {
              try
              {
                  // parse string to get April Tag configurations.
                  id = std::stoi(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                  tag_size = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                  
                  tagsize_map[id] = tag_size;
                  
                  Eigen::Matrix4d T_mat;
                  for(int i = 0 ; i < 4; ++i)
                  {
                      getline (myfile,s);
                      for(int j=0; j < 4; ++j)
                      {
                          T_mat(i,j) = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                      }

                  }

                  okvis::kinematics::Transformation temp(T_mat);
                  //std::cout << "4x4 Transformation Matrix read from file = " << temp.T() << "\n";
                  apriltagpose_map[id] = temp;                   

              }

              catch(const std::invalid_argument& e)
              {
                  std::cerr << "Invalid argument: " << e.what() << '\n';
                  ROS_ERROR("%s","Please ensure that lines in the April tag world pose config file are not incorrectly formatted(stray newlines, or format is incorrect).  \n");
              }
          }


           
      }

  
  }
  else
  {
      ROS_WARN("%s", "Could not find configuration file for April Tag(s) world pose.");
  }
 

  this->setTagSizeMap(tagsize_map);
  this->set_T_WT_map(apriltagpose_map);

}







// Note: Only gets the pose of the first detected tag.
// Input: camera parameters of a specific camera,
// returns a true if an April tag was detected (what to do if multiple are there ?)
bool MyAprilTagDetector::getTagPose_T_CT(cv::Mat img, double camera_params[], std::pair<int, okvis::kinematics::Transformation>& myPair , bool displayTags/*=false*/)
{

  

  bool foundAprilTag = false;

	// Get the list of detected tags
	cv::Mat gray_img;
  cv::cvtColor(img, gray_img, CV_BGR2GRAY);
	std::vector<AprilTags::TagDetection> detectedTags = this->tagDetector.extractTags(gray_img);


	int len = detectedTags.size() ;
	
	Eigen::Matrix4d T_CT;


  // Currently only want to detect 1 April Tag per camera, avoid confusion.
	if(len > 0)
	{

      foundAprilTag = true;

      // Just checking if received the correct camera parameters.
      //printf("\nCamera params passed to AprilTag Library =(%f,%f,%f,%f)\n",camera_params[0],camera_params[1],camera_params[2],camera_params[3]);

      int id = detectedTags[0].id;

      // TODO: add a check here for tag size.
      if( tagsize_map.find(id) != tagsize_map.end() )
      {

          double tag_size = this->tagsize_map[id];
          //std::cout << "tag size = " << tag_size << "\n";

          // NOTE: If multiple AprilTags are used, then ONLY THE FIRST ONE is used to 
          // obtain the given camera's pose. There is not paritcular reason for this 
          // choice apart from the reason so as to keep it simple. 
          // (It may be possible to average readings from multiple AprilTags ??)
    		  T_CT = detectedTags[0].getRelativeTransform(  tag_size,
                                                        camera_params[0],
                                                        camera_params[1],
                                                        camera_params[2],
                                                        camera_params[3]);

          okvis::kinematics::Transformation T_CT_transform;
    		  T_CT_transform.set(T_CT);

          // Fill output.
          myPair.first = id;
          myPair.second = T_CT_transform;

        
    	     // solely for debugging purposes.
    	    if(displayTags)
    	    {

        			cv::Mat K = getIntrinsicFromCameraParams(camera_params);
        			cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
        			R.at<double>(0,0) = T_CT(0,0);
        			R.at<double>(0,1) = T_CT(0,1);
        			R.at<double>(0,2) = T_CT(0,2);

        			R.at<double>(1,0) = T_CT(1,0);
        			R.at<double>(1,1) = T_CT(1,1);
        			R.at<double>(1,2) = T_CT(1,2);

        			R.at<double>(2,0) = T_CT(2,0);
        			R.at<double>(2,1) = T_CT(2,1);
        			R.at<double>(2,2) = T_CT(2,2);

        			cv::Mat t = cv::Mat::zeros(3, 1, CV_64FC1);
        			t.at<double>(0,0) = T_CT(0,3);
        			t.at<double>(0,1) = T_CT(1,3);
        			t.at<double>(0,2) = T_CT(2,3);


        			myDrawAxes(img, K, R, t);

        			cv::circle(img, cv::Point(detectedTags[0].p[0].first, detectedTags[0].p[0].second),
        			3,green, 2 );

        			cv::circle(img, cv::Point(detectedTags[0].p[1].first, detectedTags[0].p[1].second),
        			3,green, 2 );

        			cv::circle(img, cv::Point(detectedTags[0].p[2].first, detectedTags[0].p[2].second),
        			3,green, 2 );
        			cv::circle(img, cv::Point(detectedTags[0].p[3].first, detectedTags[0].p[3].second),
        			3,green, 2 );  

        			std::stringstream ss;
        			ss << "id =" << detectedTags[0].id ;
        			float orig_x = (detectedTags[0].p[0].first + detectedTags[0].p[1].first)/2.0 ;
        			float orig_y = (detectedTags[0].p[0].second + detectedTags[0].p[2].second)/2.0 ;
        			cv::putText(img, ss.str(), cv::Point(orig_x,orig_y),
        			          cv::FONT_HERSHEY_SIMPLEX, 0.5 , green ); 


              char* aprilTag_window = "visualization aprilTag";
              cv::namedWindow(aprilTag_window, cv::WINDOW_NORMAL);
        			cv::imshow(aprilTag_window, img);  
        			cv::waitKey(10000); // display for milliseconds.
              cv::destroyWindow(aprilTag_window);

    	    }

        }
        else
        {
          ROS_WARN("Tag size not found for April Tag with id = %d ...", id);
        }
	}

  else if(len <= 0)
  {
      ROS_WARN("%s","0 or >1 April Tag detected ...");
      foundAprilTag = false;
  }


  return foundAprilTag;

}






