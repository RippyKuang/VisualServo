#include <opencv2/opencv.hpp>


const int kMaxCorners = 4;
const double kQualityLevel = 0.1;
const double kMinDistance = 1;


void getCornerSubPix(cv::Mat &rgbMat, std::vector<cv::Point2f> &corners, bool showImage = false);