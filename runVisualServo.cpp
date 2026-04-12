#include "simulate.h"
#include "imgProcess.h"
#include "pbvs.h"
int main(int argc, const char **argv)
{
    using namespace cv;

    Simulate simulate(argv[1]);

    const char *cameraName = "endCamera";
    const std::vector<Vec3d> objectPoints = {{-0.05, -0.05, 0}, {0.05, -0.05, 0}, {0.05, 0.05, 0}, {-0.05, 0.05, 0}};

    std::vector<Point2f> corners;
    corners.reserve(kMaxCorners);

    const Matx33d K = simulate.intrinsic(cameraName);
    const Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    Mat image;
    Matx33d wRc, cam_mat;

    Vec3d cam_pos, rvec, tvec;
    Vec3d body_vel, body_angular;
    Vec3d spatial_vel, spatial_ang;

    PBVS_Control pbvs_ctrl(0.15, Vec3d{0, 0, 0.3});

    while (1)
    {

        simulate.fetchFrameInfo(image, cam_mat, cam_pos, cameraName);

        if (findKeyPoints(image, corners, false))
        {

            solvePnP(objectPoints, corners, K, distCoeffs, rvec, tvec);

            pbvs_ctrl.forward(tvec, rvec, body_vel, body_angular);

            std::cout << "tvec: " << tvec << std::endl;
            spatial_vel = cam_mat * body_vel;

            spatial_ang = cam_mat * body_angular;

            simulate.ctrl(spatial_vel, spatial_ang, cameraName);
        }

        corners.clear();
    }
    return 0;
}
