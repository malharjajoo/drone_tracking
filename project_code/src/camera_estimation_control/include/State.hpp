#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/StdVector>

#include <vector>

#include <fstream>

#include <Eigen/StdVector>
#include "kinematics/Transformation.hpp"

#include "Utils.h"
#include "globals.hpp"

class State
{

private:

	// this is used internally only, hence private.
	void setQuaternionFromEuler()
    {
        Eigen::Quaterniond q = RPYToQuaternion(degreeToRadian(this->rpy_degrees));

        setQuaternion(q);
    }

    // this is used internally only, hence private.
    void setQuaternion(Eigen::Quaterniond q)
    {
    	this->q_WS.x() = q.x();
    	this->q_WS.y() = q.y();
    	this->q_WS.z() = q.z();
    	this->q_WS.w() = q.w();
    }


public:	
	State()
	{
		r_W(0) = 0 ; r_W(1) = 0 ; r_W(2) = 0;
		
		// default rotation is identity.
		rpy_degrees(0) = 0; rpy_degrees(1) = 0; rpy_degrees(2) = 0;
		q_WS.x() = 0 ; q_WS.y() = 0 ; q_WS.z() = 0; q_WS.w() = 1;
	}

	State(double x, double y, double z, double roll, double pitch, double yaw)
	{
		r_W(0) = x ; r_W(1) = y ; r_W(2) = z;
		
		// default rotation is identity.
		rpy_degrees(0) = roll; rpy_degrees(1) = pitch; rpy_degrees(2) = yaw;
		setQuaternionFromEuler(); 
	}

	
	void setRPY_degrees(double roll, double pitch, double yaw)
    {
        rpy_degrees(0) = roll;
		rpy_degrees(1) = pitch;
		rpy_degrees(2) = yaw;

		// Update quaternions internally.
		setQuaternionFromEuler(); 
    }

    // Simple overload 
    void setRPY_degrees(Eigen::Vector3d rpy_deg)
    {
        rpy_degrees(0) = rpy_deg(0);
		rpy_degrees(1) = rpy_deg(1);
		rpy_degrees(2) = rpy_deg(2);

		// Update quaternions internally.
		setQuaternionFromEuler();
    }

    void setXYZ(const double& x, const double& y, const double& z)
    {
        r_W(0) = x;
		r_W(1) = y;
		r_W(2) = z;  
    }

    

	//world coordinates 
	Eigen::Vector3d r_W;
	Eigen::Vector3d rpy_degrees; //Euler angles in DEGREES. declared separetely for convenience..
	Eigen::Quaterniond q_WS;

	friend std::ostream& operator<<(std::ostream& os, const State& stateEstimate);
};






//=================== Summary information ================


class DroneAndCameraStatus
{

	public:
		uint64_t timestamp; // currently this is the same for all 3 cameras.
		std::vector<State> droneWorldstate_per_camera;
		std::vector<int> leds_per_camera;
		std::vector<bool> foundpose_per_camera;

		// Currently a percentage of leds_found/total_leds
		float confidence_level;
		std::vector<double> latency;


		DroneAndCameraStatus(uint64_t timestamp, 
        const std::vector<State>& droneWorldstate_per_camera,
        const std::vector<int>& leds_per_camera, 
        const std::vector<bool>& foundpose_per_camera,
        float confidence_level,
        std::vector<double> latency):

		timestamp(timestamp),
        droneWorldstate_per_camera(droneWorldstate_per_camera),
        leds_per_camera(leds_per_camera),
        foundpose_per_camera(foundpose_per_camera),
        confidence_level(confidence_level),
        latency(latency)
		{}

};







// This is created to obtain summary information of a session,
class SessionSummary
{
	private:
		std::vector<DroneAndCameraStatus> sessSummary;
		int queue_size;

	public:

		// set queue size according to how much information you'd like
		// to store per session.
		SessionSummary(int queue_size):
		queue_size(queue_size)
		{}


		void addStatus(DroneAndCameraStatus drone_camera_status)
		{

			if(sessSummary.size() < queue_size)
			{
				sessSummary.push_back(drone_camera_status);
			}
			
		}



		~SessionSummary()
		{
			
			if(sessSummary.size() > 0)
			{	

				std::cout << "Storing session summary into a CSV file ..\n";


				std::string filePath =  workspace_path + "graphs/summary.txt";
				std::ofstream _file;
				_file.open(filePath.c_str(), std::ofstream::out);
				
				if(_file.is_open())
				{
					std::string _separator = ",";

				    _file << "timestamp" <<_separator << "cam_id" << _separator << "foundPose" <<
				              _separator << "leds_per_cam" << _separator << "confidence" << _separator << "latency" << _separator << "stateEstimate" << "\n";



				    int total_cams = sessSummary.at(0).droneWorldstate_per_camera.size();

				    for(int i = 0, len = sessSummary.size(); i < len ; ++i)
				    {
				        DroneAndCameraStatus status = sessSummary[i];

				           for(int camid = 0; camid < total_cams ; ++camid)
				           {
				                _file << status.timestamp << _separator << camid << _separator << status.foundpose_per_camera[camid] <<
				                  _separator << status.leds_per_camera[camid] << _separator << status.confidence_level
				                  << _separator << status.latency[camid] << _separator << status.droneWorldstate_per_camera[camid] << std::endl;

				           }

				     }

				     _file.flush();
				     _file.close();


				}
				else
				{
					ROS_ERROR("%s", "Incorrect file path. Not storing sessiond data.");
				}

				

			}
			else
			{
				ROS_INFO("%s", "No Session summary found. Not storing in file.");
			}


			

		}






};




#endif
