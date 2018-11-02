#include "Registerer.hpp"

#include <ros/ros.h>
#include <ros/console.h>

using namespace cv;





    
// 4x4 homogenous transform from the result of the solvePnP
Eigen::Matrix4d get_T_CS(PnPProblem* pnpObj)
{
    Eigen::Matrix4d T_CS ;


    cv::Mat rotMat = pnpObj->get_R_matrix();
    cv::Mat transMat = pnpObj->get_t_matrix();

    T_CS(0,0) = rotMat.at<double>(0,0);
    T_CS(0,1) = rotMat.at<double>(0,1);
    T_CS(0,2) = rotMat.at<double>(0,2);


    T_CS(1,0) = rotMat.at<double>(1,0);
    T_CS(1,1) = rotMat.at<double>(1,1);
    T_CS(1,2) = rotMat.at<double>(1,2);


    T_CS(2,0) = rotMat.at<double>(2,0);
    T_CS(2,1) = rotMat.at<double>(2,1);
    T_CS(2,2) = rotMat.at<double>(2,2);

    T_CS(0,3) = transMat.at<double>(0);
    T_CS(1,3) = transMat.at<double>(1);
    T_CS(2,3) = transMat.at<double>(2);

    T_CS(3,0) = 0;
    T_CS(3,1) = 0;
    T_CS(3,2) = 0;
    T_CS(3,3) = 1;

    return T_CS;
}








//======================================================


Registerer::Registerer(DroneTracker* droneTracker, double camera_params[], double distortioncoeffs[],
                        Mesh* mesh, Mesh* entireMesh, int camid):
droneTracker(droneTracker),
pnpObj(camera_params, distortioncoeffs),
mesh(mesh),
entireMesh(entireMesh),
camid(camid)
{
    
   // set parameters
  this->numKeyPoints = 4000;

  //Instantiate robust matcher: detector, extractor, matcher
  // NOTE: extractor and matcher are initialized with default values and hence not shown here.
  // We only wish to pass a different ORB object for deature detection and hence explicitly set here.
  
  Ptr<FeatureDetector> detector = ORB::create(numKeyPoints);
  this->rmatcher.setFeatureDetector(detector);

  

  // Create & Open Window
  pose_mesh_window = "[Pose Estimation:"+SSTR(camid)+"] Pose-Mesh";
  
}
  
  

// This function does most of the work.
bool Registerer::findPose(cv::Mat img_in,std::pair<Eigen::Matrix4d,int>& camerapose_and_ledsfound , int displayLevel/*=0*/)
{
 
    bool computedPose = false;
    
    

    // Open the image to register
    cv::Mat img_vis = img_in.clone();


    // Get the corresponding 2D image coordinates for the 3D LED coordinates.
    std::vector<cv::Point2f> list_points2d;
    std::vector<cv::Point3f> list_points3d = mesh->getVertexList();

     

    // We pass in the 3D coordinates because of the way the LED ordering is computed.
    bool displayDetection = false;
    if(displayLevel == 1 || displayLevel == 3)
    {
        displayDetection = true;
    }
    
    droneTracker->trackLED(img_vis,list_points2d,list_points3d,displayDetection);   
   

   
    int leds_found = list_points2d.size();
    
    if(leds_found != list_points3d.size())
    {
      computedPose = false;
      //ROS_WARN("%s","# of LEDs detected (%d) != # known object points (%d)..",leds_found,list_points3d.size());
    }
  

    else
    {
        
        // OpenCV Ransac crashes if number of correspondences is < 4.
        if( leds_found >= 4 )
        {

            // COMPUTE CAMERA POSE 
            //ROS_INFO("%s","COMPUTING POSE ...");

            // SOLVEPNP_ITERATIVE- another flag for solvePnP
            bool is_correspondence = pnpObj.estimatePose(list_points3d, list_points2d, CV_EPNP);
            if ( is_correspondence )
            {
                //ROS_INFO("%s","Correspondence found");
               
                
                camerapose_and_ledsfound.first = get_T_CS(&pnpObj);
                camerapose_and_ledsfound.second = leds_found;

                computedPose = true;

                if(displayLevel == 2 || displayLevel == 3)
                {
                    // Compute all the 2D points of the mesh to verify the algorithm and draw it
                   
                    /*
                    std::vector<Point2f> list_points2d_smallmesh = pnpObj.verify_points(mesh);
                    //std::cout << "Number of 2d points =" << list_points2d_smallmesh.size() << "\n";
                    draw2DPoints(img_vis, list_points2d_smallmesh, green);


                    // check if LED centres are correct....
                    char* small_mesh_window = "Verify Known Points";
                    cv::namedWindow(small_mesh_window, cv::WINDOW_NORMAL) ;
                    cv::imshow(small_mesh_window, img_vis);
                    cv::waitKey(1);
                    */
                   

                     /*
                    // COMPUTE 3D of the image Keypoints //
                    // Containers for keypoints and descriptors of the model
                    std::vector<KeyPoint> keypoints_model;
                    cv::Mat descriptors;

                    std::cout << "\n Before computing keypoints \n";
                   
                    // Compute keypoints and descriptors
                    rmatcher.computeKeyPoints(img_in, keypoints_model);
                    rmatcher.computeDescriptors(img_in, keypoints_model, descriptors);


                    std::cout << "\n # of keypoints = " << keypoints_model.size() << "\n";
                   

                     // The list of the points2d of the model
                    std::vector<Point2f> list_points_in ;
                    std::vector<Point2f> list_points_out ;


                    // Check if keypoints are on the surface of the registration image and add to the model
                    for (unsigned int i = 0, len = keypoints_model.size();  i < len ; ++i) 
                    {
                      
                        cv::Point2f point2d(keypoints_model[i].pt);
                        cv::Point3f point3d;

                        //----> KEY function call. Uses Ray-Triagnle intersection to find inliers and 
                        // their corresponding (nearest) 3D object point.
                        bool on_surface = this->pnpObj.backproject2DPoint(&entireMesh, point2d, point3d); 

                        // add only the inliers 
                        if (on_surface)
                        {
                            list_points_in.push_back(point2d);
                        }

                        else
                        {
                            list_points_out.push_back(point2d);
                        }

                    }

               
                    img_vis = img_vis.clone();

                    // Draw some debug text
                    string num = IntToString((int)list_points_in.size());
                    string text = "There are " + num + " inliers";
                    std::cout << text <<"\n";
                    drawText(img_vis, text, green);

                    // Draw some debug text
                    num = IntToString((int)list_points_out.size());
                    text = "There are " + num + " outliers";
                    std::cout << text <<"\n";
                    drawText2(img_vis, text, red);

                    // Draw found keypoints depending on if are or not on the surface
                    draw2DPoints(img_vis, list_points_in, green);
                    draw2DPoints(img_vis, list_points_out, red);
                    */
                    
                    // Draw the object mesh
                    drawObjectMesh(img_vis, entireMesh, &pnpObj, blue);
                 
                    displayLEDOrdering(list_points2d, img_vis);
                    
                    // This can be quite useful to draw.
                    //drawAxes(img_vis,&pnpObj);
                    //myDrawAxes(img_vis, pnpObj.get_A_matrix(), pnpObj.get_R_matrix(), pnpObj.get_t_matrix());
                    myDrawAxes(img_vis, pnpObj.get_A_matrix(), camerapose_and_ledsfound.first);
                    
                    
                } 
              
            } 
            else 
            {
                computedPose = false;
                ROS_WARN("%s","Correspondence not found \n\n");
            }

        }

        else
        {
            computedPose = false;
            ROS_WARN("%s","0 or too few correspondences");
        }

    }


    if(displayLevel == 2 || displayLevel == 3)
    {
        cv::namedWindow(pose_mesh_window, WINDOW_KEEPRATIO);
        cv::imshow(pose_mesh_window, img_vis);
        cv::waitKey(1);
    }
   

            
    return computedPose;

  

  }