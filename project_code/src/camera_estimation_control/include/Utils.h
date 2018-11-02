/*
 * Utils.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "PnPProblem.h"

#include "kinematics/Transformation.hpp"
#include "kinematics/operators.hpp"
#include <Eigen/StdVector>

#include <ros/ros.h>

#include "globals.hpp"





//============== Functions added by me ============
void drawAxes(cv::Mat image,PnPProblem* pnpObj);

cv::Point2f myBackproject3DPoint(const cv::Point3f &point3d, const cv::Mat& K_matrix, const cv::Mat& R_T_concat_matrix);
void myDrawAxes(cv::Mat image, const cv::Mat& K_matrix, const cv::Mat& R_matrix, const cv::Mat& t_matrix);
void myDrawAxes(cv::Mat image,  const cv::Mat& K_matrix, Eigen::Matrix4d T_AB);


int factorial(int n);
void printVector(std::vector<int> vp);

cv::Mat getIntrinsicFromCameraParams(const double params[]);

// Just for convenience
cv::Mat EigenToCVMat(const Eigen::Matrix4d& eigenMat);

Eigen::Matrix4d CVMatToEigen(cv::Mat mat);


cv::Rect getLargestBoundingBox(cv::Mat input_image);
cv::Rect getLargestBoundingBox(cv::UMat input_image);

void displayLEDOrdering(const std::vector<cv::Point2f>& led_list, cv::Mat frame);



//======= Orientatin related added by me =======

double degreeToRadian(double degrees);
Eigen::Vector3d degreeToRadian(Eigen::Vector3d rpy_degrees);

double radianToDegrees(double radians);
Eigen::Vector3d radianToDegrees(Eigen::Vector3d rpy);

Eigen::Vector3d rotationMatrixToEulerAngles_Eig(cv::Mat R);
Eigen::Quaterniond RPYToQuaternion(Eigen::Vector3d rpy);
// Note: The orientation is returned in the order: roll, pitch and yaw.
// Also note that the input values are in radians
Eigen::Vector3d quaternionToRPY(Eigen::Quaterniond q_WS);

cv::Mat getRollPitchYawFromHomogenousMatrix(cv::Mat T_WS);

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R);

// Follows the Z-Y-X Tai bryan angle convention 
// Taken from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
cv::Mat rotationMatrixToEulerAngles(cv::Mat &R);

cv::Mat getRollPitchYawFromHomogenousMatrix(const Eigen::Matrix4d& homogenous4x4_matrix);



//===================================================


// Draw a text with the question point
void drawQuestion(cv::Mat image, cv::Point3f point, cv::Scalar color);

// Draw a text with the number of entered points
void drawText(cv::Mat image, std::string text, cv::Scalar color);

// Draw a text with the number of entered points
void drawText2(cv::Mat image, std::string text, cv::Scalar color);

void drawText3(cv::Mat image, std::string text, cv::Scalar color);

void drawText4(cv::Mat image, std::string text, cv::Scalar color, cv::Point point);

// Draw a text with the frame ratio
void drawFPS(cv::Mat image, double fps, cv::Scalar color);

// Draw a text with the frame ratio
void drawConfidence(cv::Mat image, double confidence, cv::Scalar color);

// Draw a text with the number of entered points
void drawCounter(cv::Mat image, int n, int n_max, cv::Scalar color);

// Draw the points and the coordinates
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color);

// Draw only the 2D points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color, int size=1);

// Draw an arrow into the image
void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude = 9, int thickness=1, int line_type=8, int shift=0);

// Draw the 3D coordinate axes
void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d);

// Draw the object mesh
void drawObjectMesh(cv::Mat image, const Mesh *mesh, const Eigen::Matrix4d& T_someFrameToCamera, const cv::Mat& K_matrix, cv::Scalar color=blue);
void drawObjectMesh(cv::Mat image, const Mesh *mesh, PnPProblem *pnpProblem, cv::Scalar color=blue);

// Computes the norm of the translation error
double get_translation_error(const cv::Mat &t_true, const cv::Mat &t);

// Computes the norm of the rotation error
double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R);

// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler);

// Converts a given string to an integer
int StringToInt ( const std::string &Text );

float StringToFloat ( const std::string &Text );

// Converts a given float to a string
std::string FloatToString ( float Number );

// Converts a given integer to a string
std::string IntToString ( int Number );

#endif /* UTILS_H_ */
