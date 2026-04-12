#include "pbvs.h"


void PBVS_Control::forward(const cv::Vec3d &current_tvec, const cv::Vec3d &current_rvec, cv::Vec3d &out_vel,cv::Vec3d &out_ang)
{
    out_vel = -lambda * (target_tvec - current_tvec + current_tvec.cross(current_rvec));
    out_ang = lambda * current_rvec;
}