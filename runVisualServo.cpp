#include "simulate.h"
#include "imgProcess.h"
#include "pbvs.h"
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

    Matx33d K = simulate.intrinsic(cameraName);
    Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    Mat image;
    Matx33d wRc, cam_mat;

    Vec3d cam_pos, rvec, tvec;
    Vec3d body_vel, body_angular;
    Vec3d spatial_vel, spatial_ang;
    PBVS_Control pbvs_ctrl(0.1, Vec3d{0.1, 0, 0});

    while (1)
    {

        simulate.fetchFrameInfo(image, cam_mat, cam_pos, cameraName);

        if (findKeyPoints(image, corners, false))
        {

            solvePnP(objectPoints, corners, K, distCoeffs, rvec, tvec);
            Rodrigues(rvec, wRc);

            // for (int i = 0; i < kMaxCorners; i++)
            //     ptsInCamera.push_back(wRc * objectPoints[i] + tvec);

            pbvs_ctrl.forward(tvec, rvec, body_vel, body_angular); // out_vel ,body twist ,angular-before-linear

            spatial_vel = tvec.cross(wRc * body_angular) + wRc * body_vel;
            spatial_ang = wRc * body_angular;
            
        }
        corners.clear();
        ptsInCamera.clear();
    }
    return 0;
}
