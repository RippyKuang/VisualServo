#include <opencv2/opencv.hpp>
#include <vector>


class PBVS_Control
{

private:
    double lambda;
    cv::Vec3d target_tvec;
public:
    PBVS_Control(double lbda,cv::Vec3d ttvec):lambda(lbda),target_tvec(ttvec)
    {

    }

    void forward(const cv::Vec3d &current_tvec, const cv::Vec3d &current_rvec, cv::Vec3d &out_vel,cv::Vec3d &out_ang);
    
};