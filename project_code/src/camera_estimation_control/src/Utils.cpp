/*
 * Utils.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include <iostream>

#include "PnPProblem.h"
#include "ModelRegistration.h"
#include "Utils.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// For text
int fontFace = cv::FONT_ITALIC;
double fontScale = 0.75;
int thickness_font = 2;

// For circles
int lineType = 8;
int radius = 4;
double thickness_circ = -1;



//============== Functions added by me ===========

void printVector(std::vector<int> vp)
{
    for(int i = 0, len = vp.size() ; i < len ; ++i)
    {
        std::cout << vp[i] << " ";   
    }
    std::cout << "\n";

}


int factorial(int n)
{
    if(n <= 1)
    {
        return 1;
    }
    else
    {
        return n * factorial(n-1);
    }   
}


// Just for convenience
cv::Mat EigenToCVMat(const Eigen::Matrix4d& eigenMat)
{
    cv::Mat cvMat = cv::Mat::zeros(4, 4, CV_64FC1);

    cvMat.at<double>(0,0) =  eigenMat(0,0);
    cvMat.at<double>(0,1) =  eigenMat(0,1);
    cvMat.at<double>(0,2) =  eigenMat(0,2);


    cvMat.at<double>(1,0) =  eigenMat(1,0);
    cvMat.at<double>(1,1) =  eigenMat(1,1);
    cvMat.at<double>(1,2) =  eigenMat(1,2);


    cvMat.at<double>(2,0) =  eigenMat(2,0);
    cvMat.at<double>(2,1) =  eigenMat(2,1);
    cvMat.at<double>(2,2) =  eigenMat(2,2);

    cvMat.at<double>(0,3) =  eigenMat(0,3);
    cvMat.at<double>(1,3) =  eigenMat(1,3);
    cvMat.at<double>(2,3) =  eigenMat(2,3);

    //cvMat.at<double>(3,0) = 0;
    //cvMat.at<double>(3,1) = 0;
    //cvMat.at<double>(3,2) = 0;
    cvMat.at<double>(3,3) = 1;

    return cvMat;
}

// Just for convenience
Eigen::Matrix4d CVMatToEigen(cv::Mat cvMat)
{
    Eigen::Matrix4d eigenMat;


    eigenMat(0,0) = cvMat.at<double>(0,0);
    eigenMat(0,1) = cvMat.at<double>(0,1);
    eigenMat(0,2) = cvMat.at<double>(0,2);


    eigenMat(1,0) = cvMat.at<double>(1,0);
    eigenMat(1,1) = cvMat.at<double>(1,1);
    eigenMat(1,2) = cvMat.at<double>(1,2);


    eigenMat(2,0) = cvMat.at<double>(2,0);
    eigenMat(2,1) = cvMat.at<double>(2,1);
    eigenMat(2,2) = cvMat.at<double>(2,2);

    eigenMat(0,3) = cvMat.at<double>(0,3);
    eigenMat(1,3) = cvMat.at<double>(1,3);
    eigenMat(2,3) = cvMat.at<double>(2,3);

    eigenMat(3,0) = 0;
    eigenMat(3,1) = 0;
    eigenMat(3,2) = 0;
    eigenMat(3,3) = 1;

    return eigenMat;
}



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}


// Follows the Z-Y-X Tai bryan angle convention 
// Taken from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
cv::Mat rotationMatrixToEulerAngles(cv::Mat &R)
{
 
    if(!isRotationMatrix(R))
    {
      ROS_INFO("%s", "The rotation matrix is not SO(3).\n");
      //std::cout << "R = " << R << "\n\n";
    }

     
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    
    
    cv::Mat eulers(3, 1, CV_64F);
    eulers.at<double>(0) = x;
    eulers.at<double>(1) = y;
    eulers.at<double>(2) = z;

    return eulers;
  
}


// Note: The orientation is returned in the order:roll, pitch and yaw.
// Also note that the values are in radians.
Eigen::Vector3d quaternionToRPY(Eigen::Quaterniond q_WS)
{
  
  Eigen::Matrix3d R = q_WS.normalized().toRotationMatrix();
  Eigen::Vector3d euler;

  /*
  euler = R.eulerAngles(2, 1, 0);
    */

  cv::Mat rotMat = cv::Mat::eye(3,3,CV_64F);
  // TODO: Remove this once it is working as expected.
  rotMat.at<double>(0,0) = R(0,0);
  rotMat.at<double>(0,1) = R(0,1);
  rotMat.at<double>(0,2) = R(0,2);
  
  rotMat.at<double>(1,0) = R(1,0);
  rotMat.at<double>(1,1) = R(1,1);
  rotMat.at<double>(1,2) = R(1,2);
  
  rotMat.at<double>(2,0) = R(2,0);
  rotMat.at<double>(2,1) = R(2,1);
  rotMat.at<double>(2,2) = R(2,2);
  

  euler = rotationMatrixToEulerAngles_Eig(rotMat);
  return euler;
  
  
}


// Calculates rotation matrix to euler angles (Z-Y-X convention)
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Eigen::Vector3d rotationMatrixToEulerAngles_Eig(cv::Mat R)
{
 
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Eigen::Vector3d(x, y, z);
     
     
     
}

// Note: The orientation is returned in the order: roll, pitch and yaw.
// Also note that the input values are in radians
Eigen::Quaterniond RPYToQuaternion(Eigen::Vector3d rpy)
{
  Eigen::Matrix3d R ;
  
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  R(0,0)= cos(yaw) * cos(pitch);
  R(0,1)= ( cos(yaw) * sin(pitch) * sin(roll) ) - ( cos(roll) * sin(yaw) );
  R(0,2)= ( sin(yaw) * sin(roll) ) + ( cos(yaw) * cos(roll) * sin(pitch) );

  R(1,0)= cos(pitch) * sin(yaw);
  R(1,1)= ( cos(yaw) * cos(roll) ) + ( sin(yaw) * sin(pitch) * sin(roll) );   
  R(1,2)= ( cos(roll) * sin(yaw) * sin(pitch) ) - ( cos(yaw) * sin(roll) ) ;
  
  R(2,0)= -1 * sin(pitch);
  R(2,1)= cos(pitch) * sin(roll);
  R(2,2)= cos(pitch) * cos(roll);
  
  Eigen::Quaterniond q_av(R);
  q_av.normalize();
    
/*
    std::cout << "The rpy (radian) =" << rpy(0) << "," <<  rpy(1)  << "," <<  rpy(2)   << "\n"; 
    std::cout << "The corresponding Rotation Matrix is " << R << "\n"; 
     std::cout << "\n Corresponding quaternions " << q_av.x() << "," << q_av.y() << "," << q_av.z() << "," << q_av.w() << "\n"; 
*/

  return q_av;
}


double degreeToRadian(double degrees)
{
  return degrees*(M_PI/180.0);
}

Eigen::Vector3d degreeToRadian(Eigen::Vector3d rpy_degrees)
{
  return Eigen::Vector3d( degreeToRadian(rpy_degrees(0)), degreeToRadian(rpy_degrees(1)) , degreeToRadian(rpy_degrees(2)) );
}


Eigen::Vector3d radianToDegrees(Eigen::Vector3d rpy)
{
  return Eigen::Vector3d( radianToDegrees(rpy(0)), radianToDegrees(rpy(1)) , radianToDegrees(rpy(2)) );
}



double radianToDegrees(double radians)
{
  return radians * double(180.0)/double(M_PI);
}


cv::Mat getRollPitchYawFromHomogenousMatrix(const Eigen::Matrix4d& T_WS)
{

    cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64FC1);

    rotMat.at<double>(0,0) = T_WS(0,0);
    rotMat.at<double>(0,1) = T_WS(0,1);
    rotMat.at<double>(0,2) = T_WS(0,2);


    rotMat.at<double>(1,0) = T_WS(1,0);
    rotMat.at<double>(1,1) = T_WS(1,1);
    rotMat.at<double>(1,2) = T_WS(1,2);


    rotMat.at<double>(2,0) = T_WS(2,0);
    rotMat.at<double>(2,1) = T_WS(2,1);
    rotMat.at<double>(2,2) = T_WS(2,2);

    cv::Mat eulers(3, 1, CV_64F);
    eulers = rotationMatrixToEulerAngles(rotMat);
   
    return eulers;

}
  



cv::Mat getRollPitchYawFromHomogenousMatrix(cv::Mat T_WS)
{

    cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64FC1);

    rotMat.at<double>(0,0) = T_WS.at<double>(0,0);
    rotMat.at<double>(0,1) = T_WS.at<double>(0,1);
    rotMat.at<double>(0,2) = T_WS.at<double>(0,2);


    rotMat.at<double>(1,0) = T_WS.at<double>(1,0);
    rotMat.at<double>(1,1) = T_WS.at<double>(1,1);
    rotMat.at<double>(1,2) = T_WS.at<double>(1,2);


    rotMat.at<double>(2,0) = T_WS.at<double>(2,0);
    rotMat.at<double>(2,1) = T_WS.at<double>(2,1);
    rotMat.at<double>(2,2) = T_WS.at<double>(2,2);

    cv::Mat eulers(3, 1, CV_64F);
    eulers = rotationMatrixToEulerAngles(rotMat);
   
    return eulers;
}
  



double radianToDegree(double radians)
{
  return radians * double(180.0)/double(M_PI);
}


cv::Mat getIntrinsicFromCameraParams(const double params[])
{
  cv::Mat K;
  K = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
  K.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
  K.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
  K.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
  K.at<double>(1, 2) = params[3];
  K.at<double>(2, 2) = 1;

  return K;
}



cv::Point2f myBackproject3DPoint(const cv::Point3f &point3d, const cv::Mat& K_matrix, const cv::Mat& R_T_concat_matrix)
{
  // 3D point vector [x y z 1]'
  cv::Mat point3d_vec = cv::Mat(4, 1, CV_64FC1);
  point3d_vec.at<double>(0) = point3d.x;
  point3d_vec.at<double>(1) = point3d.y;
  point3d_vec.at<double>(2) = point3d.z;
  point3d_vec.at<double>(3) = 1;

  // 2D point vector [u v 1]'
  cv::Mat point2d_vec = cv::Mat(4, 1, CV_64FC1);
  point2d_vec = K_matrix * R_T_concat_matrix * point3d_vec;

  // Normalization of [u v]'
  cv::Point2f point2d;
  point2d.x = (float)(point2d_vec.at<double>(0) / point2d_vec.at<double>(2));
  point2d.y = (float)(point2d_vec.at<double>(1) / point2d_vec.at<double>(2));

  return point2d;
}





void displayLEDOrdering(const std::vector<cv::Point2f>& led_list, cv::Mat frame)
{
    // draw all the points in the respective quadrants
   

    for(int i=0, len = led_list.size(); i < len ; ++i )
    {
        cv::Point2f pt;
        pt.x = led_list[i].x;
        pt.y = led_list[i].y;
        drawText4(frame, SSTR(i)+"led", red, pt);
    }

}



cv::Rect getLargestBoundingBox(cv::UMat input_image)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(input_image,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    int contour_len = contours.size();
    

    cv::Rect largestRect;

    float max_area = -1;

    for(int i=0; i < contour_len ; ++i)
    {
        cv::Rect rect = cv::boundingRect( cv::Mat(contours[i]) );

        if( rect.area() > max_area )
        {
            max_area = rect.area();
            largestRect = rect;
        }

    }




    return largestRect;

}





cv::Rect getLargestBoundingBox(cv::Mat input_image)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(input_image,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    int contour_len = contours.size();
    
    cv::Rect largestRect;

    float max_area = -1;

    for(int i=0; i < contour_len ; ++i)
    {
        cv::Rect rect = cv::boundingRect( cv::Mat(contours[i]) );

        if( rect.area() > max_area )
        {
            max_area = rect.area();
            largestRect = rect;
        }

    }


    return largestRect;

}


//=================================



// Draw a text with the question point
void drawQuestion(cv::Mat image, cv::Point3f point, cv::Scalar color)
{
  std::string x = FloatToString(point.x);
  std::string y = FloatToString(point.y);
  std::string z = FloatToString(point.z);

  std::string text = " Where is point (" + x + ","  + y + "," + z + ") ?";
  cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText(cv::Mat image, std::string text, cv::Scalar color)
{
  cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText2(cv::Mat image, std::string text, cv::Scalar color)
{
  cv::putText(image, text, cv::Point(25,75), fontFace, fontScale, color, thickness_font, 8);
}

void drawText3(cv::Mat image, std::string text, cv::Scalar color)
{
  cv::putText(image, text, cv::Point(25,100), fontFace, fontScale, color, thickness_font, 8);
}

void drawText4(cv::Mat image, std::string text, cv::Scalar color, cv::Point point)
{
  cv::putText(image, text, point, fontFace, fontScale, color, thickness_font, 5);
}


// Draw a text with the frame ratio
void drawFPS(cv::Mat image, double fps, cv::Scalar color)
{
  std::string fps_str = IntToString((int)fps);
  std::string text = fps_str + " FPS";
  cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the frame ratio
void drawConfidence(cv::Mat image, double confidence, cv::Scalar color)
{
  std::string conf_str = IntToString((int)confidence);
  std::string text = conf_str + " %";
  cv::putText(image, text, cv::Point(500,75), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawCounter(cv::Mat image, int n, int n_max, cv::Scalar color)
{
  std::string n_str = IntToString(n);
  std::string n_max_str = IntToString(n_max);
  std::string text = n_str + " of " + n_max_str + " points";
  cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw the points and the coordinates
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color)
{
  for (unsigned int i = 0; i < list_points_2d.size(); ++i)
  {
    cv::Point2f point_2d = list_points_2d[i];
    cv::Point3f point_3d = list_points_3d[i];

    // Draw Selected points
    cv::circle(image, point_2d, radius, color, -1, lineType );

    std::string idx = IntToString(i+1);
    std::string x = FloatToString(point_3d.x);
    std::string y = FloatToString(point_3d.y);
    std::string z = FloatToString(point_3d.z);
    std::string text = "P" + idx + " (" + x + "," + y + "," + z +")";

    point_2d.x = point_2d.x + 10;
    point_2d.y = point_2d.y - 10;
    cv::putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
  }
}

// Draw only the 2D points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color, int size/*=1*/)
{
  for( size_t i = 0; i < list_points.size(); i++)
  {
    cv::Point2f point_2d = list_points[i];

    // Draw Selected points
    cv::circle(image, point_2d, radius, color, size, lineType );
  }
}

// Draw an arrow into the image
void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
  //Draw the principle line
  cv::line(image, p, q, color, thickness, line_type, shift);
  const double PI = CV_PI;
  //compute the angle alpha
  double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
  //compute the coordinates of the first segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
  //Draw the first segment
  cv::line(image, p, q, color, thickness, line_type, shift);
  //compute the coordinates of the second segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
  //Draw the second segment
  cv::line(image, p, q, color, thickness, line_type, shift);
}


void drawAxes(cv::Mat image, PnPProblem* pnpObj)
{
    float l = 0.2; // 20cm
    std::vector<cv::Point2f> pose_points2d;
    pose_points2d.push_back(pnpObj->backproject3DPoint(cv::Point3f(0,0,0)));  // axis center
    pose_points2d.push_back(pnpObj->backproject3DPoint(cv::Point3f(l,0,0)));  // axis x
    pose_points2d.push_back(pnpObj->backproject3DPoint(cv::Point3f(0,l,0)));  // axis y
    pose_points2d.push_back(pnpObj->backproject3DPoint(cv::Point3f(0,0,l)));  // axis z
    draw3DCoordinateAxes(image, pose_points2d);           // draw ax
}



void myDrawAxes(cv::Mat image, const cv::Mat& K_matrix, const cv::Mat& R_matrix, const cv::Mat& t_matrix)
{
    
    cv::Mat R_T_concat_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
    R_T_concat_matrix.at<double>(0,0) = R_matrix.at<double>(0,0);
    R_T_concat_matrix.at<double>(0,1) = R_matrix.at<double>(0,1);
    R_T_concat_matrix.at<double>(0,2) = R_matrix.at<double>(0,2);

    R_T_concat_matrix.at<double>(1,0) = R_matrix.at<double>(1,0);
    R_T_concat_matrix.at<double>(1,1) = R_matrix.at<double>(1,1);
    R_T_concat_matrix.at<double>(1,2) = R_matrix.at<double>(1,2);

    R_T_concat_matrix.at<double>(2,0) = R_matrix.at<double>(2,0);
    R_T_concat_matrix.at<double>(2,1) = R_matrix.at<double>(2,1);
    R_T_concat_matrix.at<double>(2,2) = R_matrix.at<double>(2,2);

    R_T_concat_matrix.at<double>(0,3) = t_matrix.at<double>(0,0);
    R_T_concat_matrix.at<double>(1,3) = t_matrix.at<double>(0,1);
    R_T_concat_matrix.at<double>(2,3) = t_matrix.at<double>(0,2);

    //std::cout << "R_T_concatmatrix=" << R_T_concat_matrix << "\n";
    //std::cout << "K_matrix=" << K_matrix << "\n";

    float l = 0.3;
    std::vector<cv::Point2f> pose_points2d;
    pose_points2d.push_back(myBackproject3DPoint(cv::Point3f(0,0,0), K_matrix, R_T_concat_matrix));  // axis center
    pose_points2d.push_back(myBackproject3DPoint(cv::Point3f(l,0,0), K_matrix, R_T_concat_matrix));  // axis x
    pose_points2d.push_back(myBackproject3DPoint(cv::Point3f(0,l,0), K_matrix, R_T_concat_matrix));  // axis y
    pose_points2d.push_back(myBackproject3DPoint(cv::Point3f(0,0,l), K_matrix, R_T_concat_matrix));  // axis z
    draw3DCoordinateAxes(image, pose_points2d);           // draw ax
}




void myDrawAxes(cv::Mat image, const cv::Mat& K_matrix, Eigen::Matrix4d T_AB)
{
    cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64FC1);

    R.at<double>(0,0) = T_AB(0,0);
    R.at<double>(0,1) = T_AB(0,1);
    R.at<double>(0,2) = T_AB(0,2);

    R.at<double>(1,0) = T_AB(1,0);
    R.at<double>(1,1) = T_AB(1,1);
    R.at<double>(1,2) = T_AB(1,2);

    R.at<double>(2,0) = T_AB(2,0);
    R.at<double>(2,1) = T_AB(2,1);
    R.at<double>(2,2) = T_AB(2,2);

    t.at<double>(0,0) = T_AB(0,3);
    t.at<double>(0,1) = T_AB(1,3);
    t.at<double>(0,2) = T_AB(2,3);

 
    myDrawAxes(image, K_matrix, R,t);

}





// Draw the 3D coordinate axes
void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d)
{
  cv::Scalar red(0, 0, 255);
  cv::Scalar green(0,255,0);
  cv::Scalar blue(255,0,0);
  cv::Scalar black(0,0,0);

  cv::Point2i origin = list_points2d[0];
  cv::Point2i pointX = list_points2d[1];
  cv::Point2i pointY = list_points2d[2];
  cv::Point2i pointZ = list_points2d[3];

  //std::cout << "\norigin =" << origin << "\n";
  //std::cout << "\nPoint X =" << pointX << "\n";
  //std::cout << "\npoint Y =" << pointY << "\n";
  //std::cout << "\nPoint z =" << pointZ << "\n";
  drawArrow(image, origin, pointX, red, 9, 2);
  drawArrow(image, origin, pointY, green, 9, 2);
  drawArrow(image, origin, pointZ, yellow, 9, 2);
  cv::circle(image, origin, radius/2, black, -1, lineType );

}

// Draw the object mesh
void drawObjectMesh(cv::Mat image, const Mesh *mesh, PnPProblem *pnpProblem, cv::Scalar color)
{
  std::vector<std::vector<int> > list_triangles = mesh->getTrianglesList();
  for( size_t i = 0; i < list_triangles.size(); i++)
  {
    std::vector<int> tmp_triangle = list_triangles.at(i);

    cv::Point3f point_3d_0 = mesh->getVertex(tmp_triangle[0]) ;
    cv::Point3f point_3d_1 = mesh->getVertex(tmp_triangle[1]) ;
    cv::Point3f point_3d_2 = mesh->getVertex(tmp_triangle[2]) ;

    cv::Point2f point_2d_0 = pnpProblem->backproject3DPoint(point_3d_0);
    cv::Point2f point_2d_1 = pnpProblem->backproject3DPoint(point_3d_1);
    cv::Point2f point_2d_2 = pnpProblem->backproject3DPoint(point_3d_2);

    cv::line(image, point_2d_0, point_2d_1, color, 1);
    cv::line(image, point_2d_1, point_2d_2, color, 1);
    cv::line(image, point_2d_2, point_2d_0, color, 1);

  }
}




cv::Mat EigenToCVMat_2ndversion(Eigen::MatrixXd eigenMat)
{
    cv::Mat cvMat = cv::Mat::zeros(eigenMat.rows(), eigenMat.cols(), CV_64FC1);

    for(int i = 0; i < eigenMat.rows(); ++i)
    {
        for(int j = 0; j < eigenMat.cols(); ++j)
        {
            cvMat.at<double>(i,j) = eigenMat(i,j);
        }
    }

    return cvMat;
}


// Overloaded , more generic version ...
// Input: Image
//        Mesh file,
//        Transformation from mesh point coordinates to camera.
//        Camera matrix

// Output: Coloured mesh in 2D.
void drawObjectMesh(cv::Mat image, const Mesh *mesh, const Eigen::Matrix4d& T_someFrameToCamera, const cv::Mat& K_matrix, cv::Scalar color)
{
  // Take the triangles in the mesh and project each vertex in the camera frame.
  // Then take  projecton
  
  //Extract top 3x4 part
  cv::Mat R_T_concat_matrix = EigenToCVMat_2ndversion(T_someFrameToCamera.block<3,4>(0,0));
 

  std::vector<std::vector<int> > list_triangles = mesh->getTrianglesList();
  for( size_t i = 0, len = list_triangles.size(); i < len ; ++i)
  {

    std::vector<int> tmp_triangle = list_triangles[i];

    // This is in drone's frame (r_drone)
    cv::Point3f point_3d_0 = mesh->getVertex(tmp_triangle[0]) ;
    cv::Point3f point_3d_1 = mesh->getVertex(tmp_triangle[1]) ;
    cv::Point3f point_3d_2 = mesh->getVertex(tmp_triangle[2]) ;


    cv::Point2f point_2d_0 = myBackproject3DPoint(point_3d_0, K_matrix, R_T_concat_matrix);
    cv::Point2f point_2d_1 = myBackproject3DPoint(point_3d_1, K_matrix, R_T_concat_matrix);
    cv::Point2f point_2d_2 = myBackproject3DPoint(point_3d_2, K_matrix, R_T_concat_matrix);

  
    
    cv::line(image, point_2d_0, point_2d_1, color, 1);
    cv::line(image, point_2d_1, point_2d_2, color, 1);
    cv::line(image, point_2d_2, point_2d_0, color, 1);

  }

}






// Computes the norm of the translation error
double get_translation_error(const cv::Mat &t_true, const cv::Mat &t)
{
  return cv::norm( t_true - t );
}

// Computes the norm of the rotation error
double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R)
{
  cv::Mat error_vec, error_mat;
  error_mat = R_true * cv::Mat(R.inv()).mul(-1);
  cv::Rodrigues(error_mat, error_vec);

  return cv::norm(error_vec);
}

// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

// Converts a given Euler angles to Rotation Matrix
// NOTE: This is following Y-Z-X Tait Bryan angles.
cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_64F);

  double x = euler.at<double>(0);
  double y = euler.at<double>(1);
  double z = euler.at<double>(2);

  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<double>(0,0) = m00;
  rotationMatrix.at<double>(0,1) = m01;
  rotationMatrix.at<double>(0,2) = m02;
  rotationMatrix.at<double>(1,0) = m10;
  rotationMatrix.at<double>(1,1) = m11;
  rotationMatrix.at<double>(1,2) = m12;
  rotationMatrix.at<double>(2,0) = m20;
  rotationMatrix.at<double>(2,1) = m21;
  rotationMatrix.at<double>(2,2) = m22;

  return rotationMatrix;
}

// Converts a given string to an integer
int StringToInt ( const std::string &Text )
{
   std::istringstream ss(Text);
   int result;
   return ss >> result ? result : 0;
}

float StringToFloat ( const std::string &Text )
{
   std::istringstream ss(Text);
   float result;
   return ss >> result ? result : 0;
}


// Converts a given float to a string
std::string FloatToString ( float Number )
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

// Converts a given integer to a string
std::string IntToString ( int Number )
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}