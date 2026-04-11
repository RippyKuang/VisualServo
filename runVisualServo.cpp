#include "simulate.h"
#include "imgProcess.h"

int main(int argc, const char **argv)
{
    using namespace cv;

    Simulate simulate(argv[1]);

    const char *cameraName = "endCamera";

    std::vector<Point2f> corners;
    std::vector<Vec3d> objectPoints;
    std::vector<Vec3d> ptsInCamera;

    objectPoints.emplace_back(0.0, -0.1, 0.1);
    objectPoints.emplace_back(0.0, 0.1, 0.1);
    objectPoints.emplace_back(0.0, 0.1, -0.1);
    objectPoints.emplace_back(0.0, -0.1, -0.1);

    corners.reserve(kMaxCorners);

    Mat K = simulate.intrinsic(cameraName);
    Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    Mat image, cam_mat, cam_pos;
    Matx33d wRc;

    while (1)
    {

        simulate.fetchFrameInfo(image, cam_mat, cam_pos, cameraName);

        if (findKeyPoints(image, corners, true))
        {

            Vec3d rvec, tvec;
            solvePnP(objectPoints, corners, K, distCoeffs, rvec, tvec);
            Rodrigues(rvec, wRc);

            for (int i = 0; i < kMaxCorners; i++)
                ptsInCamera.push_back(wRc * objectPoints[i] + tvec);
            cv::imshow("aa",image);
            cv::waitKey(1);
        }
        corners.clear();
        ptsInCamera.clear();
    }
    return 0;
}
