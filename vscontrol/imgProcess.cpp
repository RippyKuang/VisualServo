#include "imgProcess.h"

using namespace cv;

const int hmin = 0, smin = 43, vmin = 46;
const int hmax = 20, smax = 255, vmax = 255;

const cv::Scalar lower(hmin, smin, vmin);
const Scalar upper(hmax, smax, vmax);

Mat hsvMat, mask;

bool findKeyPoints(Mat &rgbMat, std::vector<cv::Point2f> &corners, bool showImage)
{
    cvtColor(rgbMat, hsvMat, cv::COLOR_BGR2HSV);
    inRange(hsvMat, lower, upper, mask);

    goodFeaturesToTrack(mask, corners, kMaxCorners, kQualityLevel, kMinDistance);

    if (corners.size() != kMaxCorners)
    {
        return false;
    }
    if (showImage)
    {
        for (size_t i = 0; i < corners.size(); i++)
            circle(rgbMat, corners[i], 2.5, Scalar(255, 0, 0), 1);

        imshow("corner Image", rgbMat);
        waitKey(1);
    }

    TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
    cornerSubPix(mask, corners, Size(5, 5), Size(-1, -1), criteria);

    return 1;
}