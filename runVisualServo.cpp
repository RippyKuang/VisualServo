#include "simulate.h"
#include "imgProcess.h"

int main(int argc, const char **argv)
{
    using namespace cv;

    Simulate simulate(argv[1]);
    Mat image;

    std::vector<Point2f> corners;
    std::vector<Point3d> objectPoints;

    objectPoints.emplace_back(0.0, -0.1, 0.1);
    objectPoints.emplace_back(0.0, 0.1, 0.1);
    objectPoints.emplace_back(0.0, 0.1, -0.1);
    objectPoints.emplace_back(0.0, -0.1, -0.1);

    corners.reserve(kMaxCorners);

    Mat K = simulate.intrinsic("endCamera");
    Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    while (1)
    {
        // time_point cpu_start = Clock::now();
        simulate.getImage(image);
       
        if (findKeyPoints(image, corners, true))
        {

            Mat rvec, tvec;
            solvePnP(objectPoints, corners, K, distCoeffs, rvec, tvec);
            
        }
        corners.clear();

      //  std::cout<<"elapsed :"<<std::chrono::duration<double>(Clock::now() - cpu_start).count()<<std::endl;
    }
    return 0;
}
