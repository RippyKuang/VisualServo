#include <opencv2/opencv.hpp>
#include <vector>

class PBVS_Control
{

private:
    double lambda;
    cv::Mat target_tvec;
public:
    PBVS_Control(double lbda,cv::Mat ttvec):lambda(lbda),target_tvec(ttvec)
    {

    }

    void forward(cv::Vec3d current_tvec,cv::Vec3d current_rvec,cv::Vec6d outVelocity);
    
};