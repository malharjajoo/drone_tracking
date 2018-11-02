/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <controller/Autopilot.hpp>
#include <stdexcept>      // std::invalid_argument

#include "Utils.h"

 


 

// globals, just in case we want to fine tune PID values at runtime.
int pitch_K_p;
int pitch_K_i;  
int pitch_K_d; 

int Roll_K_p;
int Roll_K_i; 
int Roll_K_d;   

int Vertical_K_p; 
int Vertical_K_i;  
int Vertical_K_d;  

int yawrate_K_p; 
int yawrate_K_i;  
int yawrate_K_d;  






namespace controller {



Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{

    // Ensure drone starts in manual mode ...
    isAutomatic_ = false;

    // receive navdata
    subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                         this);

    // commands
    pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
    pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
    pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // flattrim service
    srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
    nh_->resolveName("ardrone/flattrim"), 1);


    // dynamically control PID parameters.
    //setInitialPIDValues_Trackbar(); createPIDTrackBars();

    setInitialPIDValues(); 
   
}





void Autopilot::setInitialPIDValues_Trackbar()
{
    pitch_K_p = 5;
    pitch_K_i = 0.00;  
    pitch_K_d = 0.5; 

    Roll_K_p = 5;
    Roll_K_i = 0.00; 
    Roll_K_d = 0.5;   

    Vertical_K_p = 10; 
    Vertical_K_i = 0.0;  
    Vertical_K_d = 0.0;  
 
    yawrate_K_p = 15; 
    yawrate_K_i = 0.0;  
    yawrate_K_d = 0.0;

    setPIDParams_Trackbar();     
}



void Autopilot::setInitialPIDValues()
{
       //  set PID constants to reasonable default values.
    controller::PidController::Parameters parameters;

    //double division_factor = 10.0;
    parameters.k_p = 0.1;
    parameters.k_i = 0.0;  
    parameters.k_d = 0.05; 
    this->pidPitch_.setParameters(parameters); 

    parameters.k_p = 0.1;
    parameters.k_i = 0.0;  
    parameters.k_d = 0.05; 
    this->pidRoll_.setParameters(parameters); 

    parameters.k_p = 1.0; 
    parameters.k_i = 0.0;  
    parameters.k_d = 0.0;
    this->pidVerticalSpeed_.setParameters(parameters); 


    // TODO: Check why this is so high ? 
    parameters.k_p = 1.5; 
    parameters.k_i = 0.0;  
    parameters.k_d = 0.0;
    this->pidYawRate_.setParameters(parameters);

}






void Autopilot::createPIDTrackBars()
{
    char* trackBarWindowName = "PID_trackBar_window" ;
    // Use only when DEBUGGING HSV/other colour space values
    cv::namedWindow(trackBarWindowName,CV_WINDOW_AUTOSIZE);
    

    cv::createTrackbar("pitch_K_p","PID_trackBar_window",&pitch_K_p, 20,onChange, this);
    cv::createTrackbar("pitch_K_i","PID_trackBar_window",&pitch_K_i, 10,onChange, this) ;
    cv::createTrackbar("pitch_K_d","PID_trackBar_window",&pitch_K_d, 20,onChange, this) ;

    cv::createTrackbar("Roll_K_p","PID_trackBar_window",&Roll_K_p, 20,onChange, this);
    cv::createTrackbar("Roll_K_i","PID_trackBar_window",&Roll_K_i, 10,onChange, this) ;
    cv::createTrackbar("Roll_K_d","PID_trackBar_window",&Roll_K_d, 20,onChange, this) ;
 
    cv::createTrackbar("Vertical_K_p","PID_trackBar_window",&Vertical_K_p, 20, onChange, this);
    cv::createTrackbar("Vertical_K_i","PID_trackBar_window",&Vertical_K_i, 10, onChange, this) ;
    cv::createTrackbar("Vertical_K_d","PID_trackBar_window",&Vertical_K_d, 20, onChange, this) ;

    cv::createTrackbar("yawrate_K_p","PID_trackBar_window",&yawrate_K_p, 20, onChange, this);
    cv::createTrackbar("yawrate_K_i","PID_trackBar_window",&yawrate_K_i, 10, onChange, this) ;
    cv::createTrackbar("yawrate_K_d","PID_trackBar_window",&yawrate_K_d, 20, onChange, this) ;

}


void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}


// Get the battery percent.
float Autopilot::batteryStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return navdata.batteryPercent;
}

// return a boolean ?
void Autopilot::hover()
{
    move(0, 0, 0, 0);
    
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;

    pubLand_.publish(landMsg);

    std::cout << "Going to land...                       status=" << status << "\n";

    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double f, double l, double u, double r)
{
  
  geometry_msgs::Twist twist;
  twist.linear.x = f;
  twist.linear.z = u;
  twist.linear.y = l;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = r;
  pubMove_.publish(twist);
  return true;
}


// Make these thread safe for now...
// Set to automatic control mode.
void Autopilot::setManual()
{
  //std::unique_lock<std::mutex> mlock(mutex_);
    isAutomatic_ = false;

}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}



// only for debugging purposes ...
void Autopilot::printWaypointList()
{
    std::cout << "\nWaypoints are ...\n";
    for(int i=0, len = waypoints_.size(); i < len ; ++i)
    {
	    Waypoint waypoint = this->waypoints_[i];
        printf("waypoint = (%f,%f,%f,%f,%f)\n", waypoint.x,  waypoint.y, waypoint.z, waypoint.yaw, waypoint.posTolerance);
    }

    
}



// Read reference waypoint list from a file
// Change this to read from a XML file ?
void Autopilot::createWaypointList(std::string filename)
{
    //bool ok = false; //f or chekc9ing if file was read correctly.

    std::ifstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        std::string s;
        std::string delimiter = " ";
        
        double x,y,z,yaw,posTolerance;


        while ( getline (myfile,s) )
        {
            // If not a comment
            if(s[0] != '#')
            {
                try
                {
            
                    // parse string to get waypoints + tolerance values
                    x = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length()); 
                    y = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                    z = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                    yaw = degreeToRadian(stod(s.substr(0, s.find(delimiter)) )) ;  s.erase(0, s.find(delimiter) + delimiter.length());
                    posTolerance = stod(s.substr(0, s.find(delimiter)) );  s.erase(0, s.find(delimiter) + delimiter.length());
                    //printf("\n waypoint = (%f,%f,%f,%f,%f)\n", x,  y, z, yaw, posTolerance);
                    Waypoint temp(x, y, z, yaw, posTolerance);
                    this->waypoints_.push_back(temp);
                }

                catch(const std::invalid_argument& e)
                {
                    std::cerr << "Invalid argument: " << e.what() << '\n';
                    ROS_ERROR("%s","No waypoints created.\n");
                    ROS_ERROR("%s","Please ensure that lines in the waypoint specification file are not incomplete(stray newlines, or format is incomplete).  \n");
                }
            }


             
        }
	
    
    }else
    {
        std::cout << "Please provide input waypoints file \n ";
    }
   
}


// For efficiency of the drone movement, yaw error is restricted between (-pi, +pi radians)
double restrictYaw(double radian_input)
{
    // First mod with 360 degree. convert to int because modulo (see below) does not accept
    // double values (radian of 360 degree is 6.28 rad which is a double)

    int deg = (int)radianToDegrees(radian_input);
    //int deg = (int)degree;

    // mod the absolute value, modulo with negative numbers are a bit counter intuitive when doing a modulo.
    if(deg < 0)
    {
        deg = (-1*deg) % 360;
        deg = -1*deg;
    }
    else
    {
        deg = deg % 360;
    }
    
    double radian = degreeToRadian(deg);
    
    if( radian > M_PI)
    {
        radian = radian - (2*M_PI);  
        
    }
    else if(radian < (-1*M_PI))
    {
        radian = (2*M_PI) + radian;
    }
    
    return radian;
}




void Autopilot::setPIDParams_Trackbar()
{

    //  set PID constants to reasonable default values.
    controller::PidController::Parameters parameters;

    double division_factor = 10.0;
    parameters.k_p = pitch_K_p/division_factor;
    parameters.k_i = pitch_K_i/division_factor;  
    parameters.k_d = pitch_K_d/division_factor; 
    this->pidPitch_.setParameters(parameters); 

    parameters.k_p = Roll_K_p/division_factor;
    parameters.k_i = Roll_K_i/division_factor;  
    parameters.k_d = Roll_K_d/division_factor; 
    this->pidRoll_.setParameters(parameters); 

    parameters.k_p = Vertical_K_p/division_factor; 
    parameters.k_i = Vertical_K_i/division_factor;  
    parameters.k_d = Vertical_K_d/division_factor;
    this->pidVerticalSpeed_.setParameters(parameters); 


    // TODO: Check why this is so high ? 
    parameters.k_p = yawrate_K_p/division_factor; 
    parameters.k_i = yawrate_K_i/division_factor;  
    parameters.k_d = yawrate_K_d/division_factor;
    this->pidYawRate_.setParameters(parameters);

}

// TODO: Put this in an infinite loop in a separate thread. The loop should check a threadsafe queue
//       for state estimates filled in by the state estimator ...
void Autopilot::pidControl(uint64_t timestampMicroseconds, State stateEstimate)
{

    

    // Do nothing if not in automatic mode ...
    if(isAutomatic_)
    {
       
        if(this->waypoints_.empty())
        {
            // Maybe just hoever and then land ? 
            // TODO: get status and then send command for landing ...
            ROS_INFO("%s", " !!! Now Landing .....\n");

             //Command to hover and land...
            //this->hover();
            this->land(); 
        }

        // If waypoint list is non-empty.
        // Check if you have reached a waypoint, and then remove it from the list.
        else
        {
            std::lock_guard<std::mutex> l(waypointMutex_);

            Waypoint waypoint = this->waypoints_.front();
            double dist =   sqrt ( (waypoint.x - stateEstimate.r_W(0) ) * (waypoint.x - stateEstimate.r_W(0)) 
                                  +(waypoint.y - stateEstimate.r_W(1) ) * (waypoint.y - stateEstimate.r_W(1)) );  
                                  //+(waypoint.z - stateEstimate.r_W(2) ) * (waypoint.z - stateEstimate.r_W(2)) );


            double yaw = stateEstimate.rpy_degrees(2);
            double abs_yaw_error = std::abs(yaw - waypoint.yaw); // Note: currently this is in degrees.
            // Since state estimation is not perfect, there is some room to allow error.
            // TODO: make sure this value leaves enough room for the front camera, since we are estimating
            // the state of the drone using the sensor.
            if( dist < waypoint.posTolerance /*&& abs_yaw_error < 3*/ )
            {
                // Comment out later if not required.
                printf("Reached waypoint !! waypoint = (%f,%f,%f) \n and position of drone is (%f,%f,%f)\n", waypoint.x,  waypoint.y, waypoint.z, stateEstimate.r_W[0], stateEstimate.r_W[1], stateEstimate.r_W[2] );
                printf("\nDistance to waypoint = %f\n", dist);
                this->waypoints_.pop_front();

                if(this->waypoints_.empty())
                {
                    ROS_INFO("%s", "COMPLETED ALL WAYPOINTS !!! Now Landing .....\n");

                    //  Command to hover and land...
                    //this->hover();
                    this->land(); 
                }
               
            }


             // After removing check again if any remaining ...
            if(!this->waypoints_.empty())
            {   
            
                Waypoint waypoint = this->waypoints_.front();
             
                // 1) Compute position and yaw error signals .. 
                okvis::kinematics::Transformation T_WS(stateEstimate.r_W, stateEstimate.q_WS);
                okvis::kinematics::Transformation T_SW = T_WS.inverse(); 

                Eigen::Vector3d ref_pos_vector = Eigen::Vector3d(waypoint.x, waypoint.y, waypoint.z);
                Eigen::Vector3d error_vec = Eigen::Vector3d( ref_pos_vector(0) - stateEstimate.r_W(0),
                                                             ref_pos_vector(1) - stateEstimate.r_W(1),
                                                             ref_pos_vector(2) - stateEstimate.r_W(2) );

                // TODO: Check why this needs to be done. Why do we need to transform them to body frame ?
                Eigen::Vector3d e_t = T_SW.C() * error_vec ;
                double yaw_error = waypoint.yaw - degreeToRadian(stateEstimate.rpy_degrees(2));
                
                // For efficiency of the drone movement, yaw error is restricted between (-pi, +pi radians)
                yaw_error = restrictYaw(yaw_error);
                

                // 2) Compute error derivatives ?!
                // Change this, obtain derivate of error signal, but very noisy.
                // Can add derivative error smoothing inside control().
                Eigen::Vector3d e_dot = Eigen::Vector3d(0.0, 0.0, 0.0);


                // 3) Supply above calculated values to PID controller and get it's output
                // Set limits for PID controllers.
                double control_euler_angle_max, control_vz_max, control_yaw_max;
                nh_->getParam("/ardrone_driver/euler_angle_max", control_euler_angle_max);
                nh_->getParam("/ardrone_driver/control_vz_max", control_vz_max);
                control_vz_max = control_vz_max/(double)1000.0; // convert to metre
                nh_->getParam("/ardrone_driver/control_yaw", control_yaw_max);
        

                //std::cout << "euler_angle_max =" << control_euler_angle_max << "\n";
                //std::cout << "control_vz_max =" << control_vz_max << "\n";
                //std::cout << "control_yaw =" << control_yaw_max << "\n";


                // Set output limits for anti-reset windup (ie, respecting physical limitations of the drone)
                pidRoll_.setOutputLimits( -1*control_euler_angle_max, control_euler_angle_max);
                pidPitch_.setOutputLimits( -1*control_euler_angle_max, control_euler_angle_max);
                pidVerticalSpeed_.setOutputLimits( -1*control_vz_max, control_vz_max);
                pidYawRate_.setOutputLimits( -1*control_yaw_max, control_yaw_max);
        
                double forward    = 0.0;
                double left       = 0.0; 
                double up         = 0.0; 
                double rotateLeft = 0.0;
        
                

                

                // NOTE: they have different units !
                forward =     pidPitch_.control(timestampMicroseconds, e_t(0) , e_dot(0));          // in rad  
                left =        pidRoll_.control(timestampMicroseconds, e_t(1) , e_dot(1));           // in rad 
                up =          pidVerticalSpeed_.control(timestampMicroseconds, e_t(2) , e_dot(2));  // in m/s
                rotateLeft =  pidYawRate_.control(timestampMicroseconds, yaw_error, 0);               // in rad/sec



                //std::cout << "\n\n"; pidPitch_.printParameters();
                //pidRoll_.printParameters(); std::cout << "\n\n";

                std::cout << "State estimate received = " << "(" << stateEstimate.r_W(0) << "," << stateEstimate.r_W(1) << "," << stateEstimate.r_W(2) << "," << stateEstimate.rpy_degrees(2) << ")\n";
                std::cout << "Error computed = " << "(" << e_t(0) << "," << e_t(1) << "," << e_t(2) << "," << yaw_error << ")\n";
;

                std::cout << "Distance =" << dist << "\n";
                std::cout << "Command: [f,l,u,r]  = " << "(" << forward <<"," << left << "," << up << "," << rotateLeft << ")" << "\n\n";

                // 4) Command drone to move ..
                move(forward, left, up, 0/*rotateLeft*/);
            }

        }

    }

}




}  // namespace arp

