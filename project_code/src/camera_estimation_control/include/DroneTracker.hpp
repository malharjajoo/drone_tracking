#ifndef DRONETRACKER_HPP_
#define DRONETRACKER_HPP_

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include "opencv2/imgcodecs.hpp"


#include <opencv2/core/ocl.hpp>
#include <unordered_map>
#include <opencv2/objdetect/objdetect.hpp>

#include "opencv2/video/background_segm.hpp"

#include "PnPProblem.h"

#include <vector>
#include "tbb/parallel_for.h"
#include <algorithm> 
#include <chrono>

#include "globals.hpp"
#include "Utils.h"
#include <cmath>
#include "opencv2/bgsegm.hpp"

#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace bgsegm;




// header for other background subtractors.
//#include "opencv2/bgsegm.hpp"




class LEDDetector
{

    private:

      // which camera this belongs to. Useful for displaying information uniqeue to a camera.
      int camid ;

      int total_leds;

      cv::Scalar min_hsv;
      cv::Scalar max_hsv;

      Mesh* entireMesh;


      std::string hsv_window_name;
      std::string rectangle_window_name;
      std::string contour_window_name;
      std::string contour_window_name2;

    public:
   

      // TODO: Change this to read from a file.
      LEDDetector(Mesh* entireMesh, int total_leds, int camid);


     
      void setThresholds();
 

      //void filterLEDRectangles(std::vector<std::vector<cv::Point> > contours,std::vector<cv::Rect>& minRect);
       void filterContours( const std::vector<std::vector<cv::Point> >& contours,
                           std::vector<std::vector<cv::Point> >& contours_after_filter);

      //============ Main Detection code ============

      // Draw bounding rectangle in a coloured image:
      // Note: input_image must be a binary 8 bit image (probably as a result of thresholding).   
      std::vector<cv::Point2f> getCentre(cv::Mat input_image, cv::Mat output_image, bool display=false);



      void detectLEDs(cv::Mat frame, std::vector<cv::Point2f>& detected_led_pts, bool display=false);



      int getTotalLEDs() const
      {
        return total_leds;
      }

     

};





// BS + LED based tracker.
class DroneTracker
{

    private:
        int camid;
        std::string fg_mask_windowname;
        std::string masked_img_window;


    public:

      DroneTracker(LEDDetector* ledDetector, const double params[], const double distcoeffs[], int camid);


      // Stores all given permutations of the LEDs in a vector.
      // This is used later during detection.
      // TODO: Place this in Utils.cpp
      void generatePermutations(int total_leds);


      // TODO: 1) How to handle the case when number of LEDs detected is not the same
      // as the number of LEDs ?  How do we know our set of 3D points ?

      //       2) Parallelize this using tbb.
      void findOrdering(std::vector<cv::Point2f> detected_led_pts,
                        std::vector<cv::Point2f>& list_points2d,
                        const std::vector<cv::Point3f>& list_points3d);


      void findOrdering_TBB(std::vector<cv::Point2f> detected_led_pts,
                        std::vector<cv::Point2f>& list_points2d,
                        const std::vector<cv::Point3f>& list_points3d);




       // Function for detecting LEDs 
      // Output: map of the LEDs.
      void trackLED(cv::Mat frame, std::vector<cv::Point2f>& list_points2d, const std::vector<cv::Point3f>& list_points3d, bool display=false);


      //void updateAndThresholdForeground_mask(cv::Mat frame, cv::Mat& thresholded_mask);
      void updateAndThresholdForeground_mask(cv::Mat frame, cv::UMat& thresholded_mask);
      

      LEDDetector* ledDetector;

      
      double camera_params[4];
      double distortioncoeffs[5];
      std::vector<std::vector<int> > permutations;


      
       // Background subtraction
      cv::Ptr<cv::BackgroundSubtractor> myBackgroundSub;
      cv::Mat fgMask;
      
};

#endif


