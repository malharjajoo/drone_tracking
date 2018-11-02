#ifndef CSVWRITER_H
#define	CSVWRITER_H

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "Utils.h"


using namespace std;
using namespace cv;

class CsvWriter {
public:
  CsvWriter(const std::string &path, const std::string &separator = " ");
  ~CsvWriter();
  void writeXYZ(const std::vector<Point3f> &list_points3d);
  void writeUVXYZ(const std::vector<Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, const Mat &descriptors);



private:
  ofstream _file;
  string _separator;
  bool _isFirstTerm;
};

#endif
