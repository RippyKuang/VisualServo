#include "pbvs.h"

cv::Mat hat(cv::Vec3d v)
{
    return (cv::Mat_<double>(3, 3) << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0);
}

void PBVS_Control::forward(cv::Vec3d current_tvec, cv::Vec3d current_rvec, cv::Vec6d outVelocity)
{
}