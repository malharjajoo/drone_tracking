#ifndef REGISTERER_HPP_
#define REGISTERER_HPP_


#include "Mesh.h"
#include <opencv2/core.hpp>
#include "RobustMatcher.h"
#include "DroneTracker.hpp"

class Registerer
{

private: 
    int camid;

    std::string pose_mesh_window;
    
public:
  Registerer(DroneTracker* droneTracker, double camera_params[], double distortionCoeffs[],
             Mesh* mesh, Mesh* entireMesh, int camid);
    



  // This function does most of the work.
  bool findPose(cv::Mat img_in,std::pair<Eigen::Matrix4d,int>& camerapose_and_ledsfound , int displayLevel=0);



//========= Data Members========

     RobustMatcher rmatcher;
    
     // CREATE MODEL REGISTRATION OBJECT
     // CREATE OBJECT MESH
     // CREATE OBJECT MODEL
     // CREATE PNP OBJECT


     PnPProblem pnpObj;


     // Set the number of points to register
     int numKeyPoints;
     DroneTracker* droneTracker;

     //bool supplyInitialPose;

     Mesh* mesh;
     Mesh* entireMesh;
};

#endif