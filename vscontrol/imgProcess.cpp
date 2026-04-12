#include "imgProcess.h"
#include <algorithm>

using namespace cv;

const int hmin = 0, smin = 43, vmin = 46;
const int hmax = 20, smax = 255, vmax = 255;

const cv::Scalar lower(hmin, smin, vmin);
const Scalar upper(hmax, smax, vmax);

Mat hsvMat, mask;

bool findKeyPoints(Mat &rgbMat, std::vector<cv::Point2f> &corners, bool debug)
{
    cvtColor(rgbMat, hsvMat, cv::COLOR_BGR2HSV);
    inRange(hsvMat, lower, upper, mask);

    goodFeaturesToTrack(mask, corners, kMaxCorners, kQualityLevel, kMinDistance);

    if (corners.size() != kMaxCorners)
    {
        return false;
    }

    std::sort(corners.begin(), corners.end(), [](cv::Point2f a, cv::Point2f b)
              { return a.y < b.y; });

    if (corners[0].x > corners[1].x)
    {
        cv::Point2f temp = corners[0];
        corners[0] = corners[1];
        corners[1] = temp;
    }
    if (corners[2].x < corners[3].x)
    {
        cv::Point2f temp = corners[2];
        corners[2] = corners[3];
        corners[3] = temp;
    }

    if (debug)
    {
        circle(rgbMat, corners[0], 3, Scalar(255, 0, 0), 2);
        circle(rgbMat, corners[1], 3, Scalar(0, 255, 0), 2);
        circle(rgbMat, corners[2], 3, Scalar(0, 0, 255), 2);
        circle(rgbMat, corners[3], 3, Scalar(255, 255, 255), 2);

        imshow("corners",rgbMat);
        waitKey(1);
    }

    TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
    cornerSubPix(mask, corners, Size(5, 5), Size(-1, -1), criteria);

    return true;
}
