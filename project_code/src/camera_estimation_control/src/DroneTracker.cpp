#include "DroneTracker.hpp"

using namespace cv;
using namespace std;



#define USE_TBB 1

// A simple switch between MOG2 or MOG method
// 0 - MOG2
// 1 - MOG method
#define USE_MOG 1




//================ Helper function =======================


// sets all pixels outside the rectangle to value = 0 (black).
cv::Mat getBoxOnly(cv::Mat img, cv::Rect2d rect)
{ 
  cv::Mat boxOnly = img.clone();
  
  // set all pixels outside rectangle to 0.
  int rows = img.rows;
  int cols = img.cols;
  for(int y = 0 ; y < rows; ++y)
  {
    for(int x = 0 ;x < cols ; ++x)
    {
      if(!rect.contains(cv::Point(x,y)))
      {
        boxOnly.at<cv::Vec3b>(y, x) = boxOnly.at<cv::Vec3b>(y, x) * 0;
      }
    }
  }

  return boxOnly;
  
}



//==================================================


// TODO: Change this to read from a file.
LEDDetector::LEDDetector(Mesh* entireMesh, int total_leds, int camid)
: entireMesh(entireMesh),
 total_leds(total_leds),
 camid(camid)
{
    setThresholds();

    hsv_window_name = "[Detection: "+SSTR(camid)+"] HSV + erosion + dilation";
    rectangle_window_name="[Detection: " + SSTR(camid)+"] Rectangles";

    contour_window_name = "[Detection"+SSTR(camid)+"] Before filtering contours";
    contour_window_name2 = "[Detection"+SSTR(camid)+"] After filtering contours";

}




void LEDDetector::setThresholds()
{
    // HSV thresholds LED 
    int h_min = 0; 
    int s_min = 0;
    int v_min = 255;

    int h_max = 179; 
    int s_max = 255;
    int v_max = 255;

    this->min_hsv = cv::Scalar(h_min, s_min, v_min);
    this->max_hsv = cv::Scalar(h_max, s_max, v_max);

   
}


/*
void LEDDetector::filterLEDRectangles(std::vector<std::vector<cv::Point> > contours,
                    std::vector<cv::Rect>& minRect)
{

    int contour_length = contours.size();
    minRect.resize(contour_length);


    for(int i=0; i < contour_length ; ++i)
    {
        minRect[i] = boundingRect( Mat(contours[i]) );
    }
   

    std::cout << "# rectangles found =" << minRect.size() << "\n";
    

    // 1) Filter 1
    //duplicate the rectangles and threshold
    // tbis removes rectangles within rectangles
    for( int i = 0, len =  minRect.size(); i < len ; i++ )
    {
        minRect.push_back(minRect[i]);
    }

    cv::groupRectangles(minRect, 1, 0.2);
    std::cout << "# rectangles after filter 1=" << minRect.size() << "\n";

    // 2) Filter 2:filter out rectangles than are not nearly equal in width and height..
    if(minRect.size() > this->total_leds)
    {

        auto new_end = std::remove_if(minRect.begin(), minRect.end(), [this,minRect](cv::Rect rect){

            float ratio1 = (float)rect.width/(float)rect.height;
            float ratio2 = (float)rect.height/(float)rect.width;
            //printf("Ratio 1= %f\n", ratio1);
            //printf("Ratio 2 = %f\n", ratio2);
            bool remove_condition = ((ratio1 > 1.5) || (ratio2 > 1.5)) && ((minRect.size()-1) > total_leds);
            return  remove_condition;  

        });

        minRect.erase( new_end, minRect.end());

    }

  
    
    // 3) Filter 3 
    if(minRect.size() > this->total_leds)
    {
      // We sort in descending order 
      // since it is easier to remove elements from the back of a vector.
      
      std::sort(minRect.begin(),minRect.end(), [](cv::Rect lhs, cv::Rect rhs){

          return lhs.area() > rhs.area();
      });

     
      int extra_rectangles = minRect.size() - this->total_leds;
      // remove the extra rectangles
      for(int b = 0; b < extra_rectangles; ++b)
      {
          minRect.pop_back();
      }

    }

              
    std::cout << "# rectangles area filter 3=" << minRect.size() << "\n";
        

       

}
*/


//============ Main Detection code ============



void LEDDetector::filterContours( const std::vector<std::vector<cv::Point> >& contours, std::vector<std::vector<cv::Point> >& contours_after_filter)
{

      // Filter 1
      int vertex_threshold = 5; // remove upto 5 sided figures.
       
        //std::cout << "# contours =" << contours.size() << "\n";

       for(int i = 0, contour_len = contours.size(); i < contour_len ; ++i)
       {
          std::vector<cv::Point> approxCurve;
          double precision = 0.01 * cv::arcLength(contours[i],true);

          cv::approxPolyDP(contours[i], approxCurve, precision, true);

          int vertices = approxCurve.size();
          if( vertices > vertex_threshold )
          {
              contours_after_filter.push_back(contours[i]);
          }

       }


       // Filter 2 - Remove contours which may be inside other contours ???
       //std::vector<int> remove_list;
       //myGroupRectangles(contours_after_filter, remove_list);
       // Remove contours using indices from remove_list.
       std::vector<cv::Rect> minRect(contours_after_filter.size());
        for(int i=0; i < contours_after_filter.size() ; ++i)
        {
            minRect[i] = boundingRect( Mat(contours_after_filter[i]) );
        }

        // 2) Filter 2:filter out contours than are not nearly equal in width and height..
        if(minRect.size() > this->total_leds)
        {

            for(int i = 0, len =  minRect.size(); i < len ; ++i)
            {
                cv::Rect rect = minRect[i];
                float ratio1 = (float)rect.width/(float)rect.height;
                float ratio2 = (float)rect.height/(float)rect.width;
                //printf("Ratio 1= %f\n", ratio1);
                //printf("Ratio 2 = %f\n", ratio2);
             
                if( ((ratio1 > 1.7) || (ratio2 > 1.7)) )
                {
                    contours_after_filter.erase(contours_after_filter.begin() + i);
                } 
                
            }

        }



        /*
        // Want to find the most similar contours ....
        for(int i = 0; i < contours_after_filter.size() ; ++i)
        {
            for(int j = 0; j < contours_after_filter.size() ; ++j)
            {
                double error = cv::matchShapes(contours_after_filter[i], contours_after_filter[j],1, 0.0);
                std::cout << "Error between contours:" << i << " and " << j << "=" << error << "\n";
                {
                  cv::Mat contour_img_after_filter = cv::Mat::zeros(cv::Size(640,480), CV_8UC3 );
                  std::vector<std::vector<cv::Point> > temp;
                  temp.push_back(contours_after_filter[i]);
                  temp.push_back(contours_after_filter[j]);
                  cv::drawContours( contour_img_after_filter, temp, -1, green,3,8 );
                  cv::namedWindow("After match shapes", cv::WINDOW_NORMAL);
                  cv::imshow("After match shapes", contour_img_after_filter);
                  cv::waitKey(0);
                }
            }
            
        }
        */


      // 3) Filter 3 - filter contours by largest area.
      if(contours_after_filter.size() > this->total_leds)
      {
          // We sort in descending order 
          // since it is easier to remove elements from the back of a vector.
          
          std::sort(contours_after_filter.begin(),contours_after_filter.end(), [](std::vector<cv::Point> lhs,
                                                                                  std::vector<cv::Point> rhs){

              return cv::contourArea(lhs) > cv::contourArea(rhs);
          });

         
          int extra_rectangles = contours_after_filter.size() - this->total_leds;
          // remove the extra rectangles
          for(int b = 0; b < extra_rectangles; ++b)
          {
              contours_after_filter.pop_back();
          }

      }


}




// Draw bounding rectangle in a coloured image:
// Note: input_image must be a binary 8 bit image (probably as a result of thresholding).   
std::vector<cv::Point2f> LEDDetector::getCentre(cv::Mat input_image, cv::Mat output_image, bool display/*=true*/)
{


    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2f> centre_list;

    cv::findContours(input_image,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

    

    // If image is completely dark (no object was detected), and no contours are found.
    if(contours.size() > 0)
    {

        //std::vector<cv::Rect> minRect;
        
        /*
        if(display)
        {
             Mat drawing = Mat::zeros( input_image.size(), CV_8UC3 );
            cv::drawContours( drawing, contours, -1, red, 2, 8 );

            
            cv::namedWindow(contour_window_name, cv::WINDOW_NORMAL);
            cv::imshow(contour_window_name, drawing);cv::waitKey(1);
        }
       */


        if(contours.size() > this->total_leds)
        {

            // Filter contours based on circularity ..
            std::vector<std::vector<cv::Point> > contours_after_filter;
            filterContours( contours,contours_after_filter);

            
            /*
            if(display)
            {
                cv::Mat contour_img_after_filter = cv::Mat::zeros( input_image.size(), CV_8UC3 );
                cv::drawContours( contour_img_after_filter, contours_after_filter, -1, green,3,8 );
                
                cv::namedWindow(contour_window_name2, cv::WINDOW_NORMAL);
                cv::imshow(contour_window_name2, contour_img_after_filter);
            }
            */
            // Find centre using image moments.
            for(int i=0, len1 = contours_after_filter.size() ; i < len1; ++i)
            {
                cv::Moments m = moments(contours_after_filter[i],true);
                cv::Point2d myCentre = cv::Point2f(m.m10/m.m00, m.m01/m.m00);
                centre_list.push_back(myCentre);
               
            }

            //filterLEDRectangles(contours_after_filter, minRect); 
        }

        else
        {

            for(int i=0, len1 = contours.size() ; i < len1; ++i)
            {
                cv::Moments m = moments(contours[i],true);
                cv::Point2d myCentre = cv::Point2f(m.m10/m.m00, m.m01/m.m00);
                centre_list.push_back(myCentre);
               
            }

            //filterLEDRectangles(contours, minRect); 
        }
      

        
        /*

        for(int i=0, len1 = minRect.size() ; i < len1; ++i)
        {
            centre_list.push_back(cv::Point2f(minRect[i].x + minRect[i].width/2.0,minRect[i].y + minRect[i].height/2.0));
        }
        */
        
       
        if(display)
        {
            
            for(int i=0, len = centre_list.size(); i < len; ++i)
            {
                //cv::rectangle( output_image, minRect[i].tl(), minRect[i].br(), yellow, 3 );
                cv::circle(output_image, centre_list[i], 2, red,3);

            }

        }

    }


    return centre_list;

}



// Input: Image in which (hopefully) LED is present.
// Output: List of centres of the LEDs.
void LEDDetector::detectLEDs(cv::Mat input_img, std::vector<cv::Point2f>& centres_list, bool display/*=false*/)
{

   //1) Remove noise using blur 
    cv::Mat frame = input_img.clone();
    

    //cv::GaussianBlur(input_img, frame, cv::Size(3,3),0);
    //cv::imshow("After blur", frame); cv::waitKey(2);

    //2) convert image
    // BGR -> HSV & threshold
    
    cv::Mat hsv_frame1;
    cv::cvtColor(frame, hsv_frame1, cv::COLOR_BGR2HSV);



    cv::inRange(hsv_frame1, this->min_hsv, this->max_hsv, hsv_frame1);    
   
    
    /*
    //4) apply erosoon/dilation to remove noise.#
    //int type = cv::MORPH_RECT;
    int erosion_size= 1;
    cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    // Apply the erosion operation
    cv::UMat dst1;
    cv::UMat hsv_frame1_umat = hsv_frame1.getUMat(ACCESS_READ);
    cv::erode(hsv_frame1_umat, dst1, erode_element);
   */

     cv::UMat dst2;
    int dilation_size = 1;
    cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    cv::dilate( hsv_frame1, dst2, dilate_element );



    cv::UMat after_blur_umat;
    cv::GaussianBlur(dst2, after_blur_umat, cv::Size(3,3),0);


    cv::Mat after_blur = after_blur_umat.getMat(ACCESS_RW);

    cv::Mat img_vis = input_img.clone();
    centres_list = getCentre(after_blur,img_vis,display);
    //blobDetector(hsv_frame1,img_vis);

    //blobDetector


    if(display)
    { 
      // After HSV + erosion + dilation
      //cv::namedWindow(hsv_window_name, cv::WINDOW_NORMAL);
      //cv::imshow(hsv_window_name, dst2);

      cv::namedWindow(rectangle_window_name, cv::WINDOW_NORMAL); 
      cv::imshow(rectangle_window_name, img_vis);

      cv::waitKey(1);
    }
    

}





//========================  BackgroundSub + LED based tracker.=============






      DroneTracker::DroneTracker(LEDDetector* ledDetector, const double params[], const double distcoeffs[], int camid)
      :ledDetector(ledDetector),
      camid(camid)
      {
          // tracker related stuff.

          camera_params[0] = params[0];
          camera_params[1] = params[1];
          camera_params[2] = params[2];
          camera_params[3] = params[3];

          distortioncoeffs[0] = distcoeffs[0];
          distortioncoeffs[1] = distcoeffs[1];
          distortioncoeffs[2] = distcoeffs[2];
          distortioncoeffs[3] = distcoeffs[3];
          distortioncoeffs[4] = distcoeffs[4];

          int total_leds = ledDetector->getTotalLEDs();
          // generates total_leds! (factorial) permutations for future use.
          generatePermutations(total_leds); 


          #if USE_MOG == 1
            // Create a tracker
            this->myBackgroundSub = cv::bgsegm::createBackgroundSubtractorMOG(/*500,16,true*/); //MOG2 approach

            fg_mask_windowname = "[BS [MOG]:"+SSTR(camid)+"] fgMask_MOG";
            masked_img_window = "[BS [MOG]:"+SSTR(camid)+"] Masked image_MOG";

          #else

            // Create a tracker
            this->myBackgroundSub = cv::createBackgroundSubtractorMOG2(/*500,16,true*/); //MOG2 approach

            fg_mask_windowname = "[BS [MOG2]:"+SSTR(camid)+"] fgMask_MOG2";
            masked_img_window = "[BS [MOG2]:"+SSTR(camid)+"] Masked image_MOG2";

          #endif




          
      }



      // Stores all given permutations of the LEDs in a vector.
      // This is used later during detection.
      void DroneTracker::generatePermutations(int total_leds)
      {

          std::vector<int> arr(total_leds);
          for(int i = 0; i < total_leds; ++i)
          {
              arr[i] = i;
          }

          int arr_size = arr.size();
          // not really needed but just for safety.
          std::sort(arr.begin(), arr.begin() + arr_size);

          int total_permutations = factorial(total_leds);
          permutations.resize(total_permutations);


          int i = 0;
          do 
          {
              std::vector<int> vp;
              vp.insert(vp.begin(), arr.begin(),arr.begin() + arr_size);
              permutations[i] = vp;

              ++i;

          } while (std::next_permutation(arr.begin(), arr.begin() + arr_size ));

       // This is a way of verifying that permutations are complete.
       // This should output the sorted input list.
        //std::cout << "After loop: " << arr[0] << " " << arr[1] << " " << arr[2] << " " << arr[3] << " " <<  arr[4] << " " << arr[5]<< " " << arr[6] << " " << arr[7] <<  "\n";




      }


      // TODO: 1) How to handle the case when number of LEDs detected is not the same
      // as the number of LEDs ?  How do we know our set of 3D points ?

      //       2) Parallelize this using tbb.
      void DroneTracker::findOrdering(std::vector<cv::Point2f> detected_led_pts,
                        std::vector<cv::Point2f>& list_points2d,
                        const std::vector<cv::Point3f>& list_points3d)
      {


         // this is a LS error measure.
          // Hence can used -1 as min value since error is always squared...
          float lowest_reprojection_error = INFINITY;
          int permutation_index = -1;

          //ROS_INFO("Total permutations = %d", permutations.size());  

          
          PnPProblem myPnPObj(camera_params,distortioncoeffs);

          //auto start = std::chrono::high_resolution_clock::now();

          
          // select permutation of 8 LEDs.
          for(int i = 0, len = permutations.size() ; i< len; ++i)
          {
             
              // 1) find pose of given ordering
              std::vector<int> order = permutations[i];

              /*
              if( detected_led_pts.size() != order.size() )
              {
                  ROS_WARN("%s", "Number of LEDs detected and permutation length is not the same!");
                  ROS_WARN("%s", "In this case the corresponding 3D points will not be known !!");
              }
              */

              // assume 1 ordering of the LEDs and create a 2D list accordingly.
              std::vector<cv::Point2f> current_list_2d(order.size());

              for(int j= 0, detected_len = detected_led_pts.size() ; j < detected_len; ++j)
              {
                  current_list_2d[order[j]] = detected_led_pts[j];
              }

              bool is_correspondence = myPnPObj.estimatePose(list_points3d, current_list_2d, SOLVEPNP_ITERATIVE);


              //===== 

              if(is_correspondence)
              {

                  // 2) project all 3d points to corresponding 2d points
                  std::vector<cv::Point2f> projection_2d(list_points3d.size());
                  for(int k = 0, projection_length = projection_2d.size() ; k < projection_length ; ++k)
                  {
                      projection_2d[k] = myPnPObj.backproject3DPoint(list_points3d.at(k) );
                  }


                  // ======

                  // 3) Find reprojection error: between current permutation and 3d points projected
                  // as 2d points.
                  float reporj_err = 0;
                  for(int m = 0, reproj_len = projection_2d.size() ; m < reproj_len; ++m )
                  {
                     reporj_err = reporj_err + (current_list_2d[m].x - projection_2d[m].x)
                                    * (current_list_2d[m].x - projection_2d[m].x) 
                                    + (current_list_2d[m].y - projection_2d[m].y)
                                    * (current_list_2d[m].y - projection_2d[m].y);

                  }


                  if(reporj_err < lowest_reprojection_error)
                  {
                      lowest_reprojection_error = reporj_err;
                      permutation_index = i;
                  }


              } 
           
             

          }
        
          /* 
          auto finish = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed = finish - start;
          std::cout << "\n[Sequential] LED Ordering Elapsed time: " << elapsed.count() << " sec\n";
          */

          // Use chosen permutation (wuth least error) for filling output..
          if(permutation_index >= 0 )
          {
              //ROS_INFO("[Sequntial]Permutation with lowest error = %d", permutation_index);
              std::vector<int> order = permutations[permutation_index];

              list_points2d.resize(order.size());

              for(int p=0, len_order = order.size() ; p < len_order ; ++p)
              {
                  list_points2d[ order[p] ] = detected_led_pts[p];
              }
              

          }
          else
          {
            ROS_WARN("%s","No permutation found with lowest reproject error..");
          }



      }


      void DroneTracker::findOrdering_TBB(std::vector<cv::Point2f> detected_led_pts,
                        std::vector<cv::Point2f>& list_points2d,
                        const std::vector<cv::Point3f>& list_points3d)
      {

          int total_permutations = permutations.size();
          // this is a LS error measure.
          // Hence can used -1 as min value since error is always squared...
          std::vector<double> lowest_reprojection_error(total_permutations,-1);
      

          //ROS_INFO("Total permutations = %d", total_permutations);  

          


          //auto start = std::chrono::high_resolution_clock::now();

          // select permutation of 8 LEDs.
          //for(int i = 0 ; i< total_permutations; ++i){
          tbb::parallel_for(0, total_permutations, [&](int i){
             
              // 1) find pose of given ordering
              std::vector<int> order = permutations[i];

              // assume 1 ordering of the LEDs and arrange 2D points accordingly.
              std::vector<cv::Point2f> current_list_2d(order.size());
              for(int j= 0, detected_len = detected_led_pts.size() ; j < detected_len; ++j)
              {
                  current_list_2d[order[j]] = detected_led_pts[j];
              }

              PnPProblem myPnPObj(camera_params,distortioncoeffs);
              bool is_correspondence = false;


              if(current_list_2d.size() >= 4 )
              {
                  is_correspondence = myPnPObj.estimatePose(list_points3d, current_list_2d, SOLVEPNP_ITERATIVE);
              }
             

              //=======

              if(is_correspondence)
              {

                  // 2) project all 3d points to corresponding 2d points
                  std::vector<cv::Point2f> projection_2d(list_points3d.size());
                  for(int k = 0, projection_length = projection_2d.size() ; k < projection_length ; ++k)
                  {
                      projection_2d[k] = myPnPObj.backproject3DPoint(list_points3d.at(k) );
                  }


                  // ======

                  // 3) Find reprojection error: between current permutation and 3d points projected
                  // as 2d points.
                  double reporj_err = 0;
                  for(int m = 0, reproj_len = projection_2d.size() ; m < reproj_len; ++m )
                  {
                     reporj_err = reporj_err + (current_list_2d[m].x - projection_2d[m].x)
                                    * (current_list_2d[m].x - projection_2d[m].x) 
                                    + (current_list_2d[m].y - projection_2d[m].y)
                                    * (current_list_2d[m].y - projection_2d[m].y);

                  }

                  // simply store the values
                  lowest_reprojection_error[i] = reporj_err;
                  
              } 
           


          });
        
          // Use chosen permutation (wuth least error) for filling output..
          // Find index with minimum re-projection error.
          double min_error = INFINITY;
          int permutation_index = -1;

          for(int i=0; i < total_permutations ; ++i)
          { 
              if(lowest_reprojection_error[i] < min_error)
              {
                  min_error = lowest_reprojection_error[i];
                  permutation_index = i;
              }
          }

          if(permutation_index >= 0 )
          {
              //ROS_INFO("[TBB]Permutation with lowest error = %d", permutation_index);
              std::vector<int> order = permutations[permutation_index];

              list_points2d.resize(order.size());

              for(int p=0, len_order = order.size(), len_leds = detected_led_pts.size() ; p < len_order && p < len_leds ; ++p)
              {
                  list_points2d[ order[p] ] = detected_led_pts[p];
              }
              

          }
          else
          {
            ROS_WARN("%s","[TBB] No permutation found with lowest reproject error..");
          }



          /*    
          auto finish = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed = finish - start;
          std::cout << "\n[TBB] LED Ordering Elapsed time: " << elapsed.count() << " sec\n";
          */

      }


      /*
      void DroneTracker::updateAndThresholdForeground_mask(cv::Mat frame, cv::Mat& thresholded_mask)
      {
          // Update background mask
          
       
            this->myBackgroundSub->apply(frame, fgMask);
          

         
          // threshold it to convert from grayscale to binary.
          // Use this after background subtraction ....            
          //int type = cv::MORPH_RECT;


          /*
          //3) Try finding local maxima ?
          //cv::Mat peak_img = bhFindLocalMaximum(input_img);  
          //4) apply erosoon/dilation to remove noise.#
          //int erosion_size = 1;
          //cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT,
                                               Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               Point( erosion_size, erosion_size ) );
          //cv::Mat dst1;
         // cv::erode(fgMask, dst1, erode_element);
        


          cv::Mat dst2;
          int dilation_size = 2;
          cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT,
                                               Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                               Point( dilation_size, dilation_size ) );

          cv::dilate( fgMask, dst2, dilate_element );


          
         
          cv::Mat after_thresh;
          cv::threshold(dst2, after_thresh, 210, 255, cv::THRESH_BINARY);

          thresholded_mask = after_thresh.clone();

      }
      */


      void DroneTracker::updateAndThresholdForeground_mask(cv::Mat frame, cv::UMat& thresholded_mask)
      {
          //auto start = std::chrono::high_resolution_clock::now();

          // Update background mask
          
            this->myBackgroundSub->apply(frame, fgMask);
        
           
          /*
          auto finish = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed = finish - start;
          std::cout << "\n[BS] BS sub Elapsed time: " << elapsed.count() << " sec\n";
          */

          // threshold it to convert from grayscale to binary.
          // Use this after background subtraction ....            
          //int type = cv::MORPH_RECT;


          /*
          //3) Try finding local maxima ?
          //cv::Mat peak_img = bhFindLocalMaximum(input_img);  
          //4) apply erosoon/dilation to remove noise.#
          int erosion_size = 1;
          cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT,
                                               Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               Point( erosion_size, erosion_size ) );
          cv::UMat dst1;
          cv::erode(fgMask, dst1, erode_element.getUMat( ACCESS_READ ));
          */

          cv::UMat dst2;

          int dilation_size = 2;

          #if USE_MOG == 1
            dilation_size = 8;
          #endif 

          cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT,
                                               Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                               Point( dilation_size, dilation_size ) );

          cv::dilate( fgMask, dst2, dilate_element.getUMat( ACCESS_READ ) );


          cv::threshold(dst2, thresholded_mask, 210, 255, cv::THRESH_BINARY);

      }

      // Function for detecting LEDs 
      // Output: map of the LEDs.
      void DroneTracker::trackLED(cv::Mat frame, std::vector<cv::Point2f>& list_points2d, const std::vector<cv::Point3f>& list_points3d, bool display/*=false*/)
      {

          // Start timer
          //double timer = (double)getTickCount();


          // Do Background subtraction ...
          cv::UMat thresholded_fg_mask;
          updateAndThresholdForeground_mask(frame, thresholded_fg_mask);
         

     
          cv::UMat masked_img_umat ;
          cv::UMat grayMask;
          cv::cvtColor(thresholded_fg_mask, grayMask, CV_GRAY2BGR);
          cv::bitwise_and(frame, grayMask, masked_img_umat);
          cv::Mat masked_img = masked_img_umat.getMat(ACCESS_RW);




          if(display)
          {
            //cv::namedWindow(fg_mask_windowname, cv::WINDOW_NORMAL);
            //cv::imshow(fg_mask_windowname, thresholded_fg_mask);

            cv::namedWindow(masked_img_window, cv::WINDOW_NORMAL);
            cv::imshow(masked_img_window, masked_img);

            cv::waitKey(1);
          }


          
          // geta bounding box around drone .. sometimes Background subtraction
          // may not detect drone completely, so need to increase width and height.
          cv::Rect temp = getLargestBoundingBox(thresholded_fg_mask);
          cv::Rect bbox( temp.x, temp.y, temp.width*1.3, temp.height*1.3);
          cv::Mat roi = getBoxOnly(masked_img, bbox);
          
         

          // Find the LEDs inside the box returned by background subtraction.
          std::vector<cv::Point2f> detected_led_pts;
          ledDetector->detectLEDs(roi, detected_led_pts, display);



          // For now the ordering algorithm assuems that all LEDs
          // are detected.
          if(detected_led_pts.size() == list_points3d.size() ) 
          {
            #if USE_TBB == 1
              findOrdering_TBB(detected_led_pts, list_points2d, list_points3d);
            #else 
              findOrdering(detected_led_pts, list_points2d, list_points3d);
            #endif

          }
          
         

          
          //float fps = getTickFrequency() / ((double)getTickCount() - timer);
          // Display FPS on frame
          //putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);


        
      }
