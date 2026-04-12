#include "robot.h"

void Robot::resetRobot()
{
    mj_resetData(m, d);
    mj_forward(m, d);
}

void Robot::moveCamera(int action, mjtNum reldx, mjtNum reldy)
{
    mjv_moveCamera(m, action, reldx, reldy, &scn, &cam);
}

void Robot::step()
{
    std::lock_guard<std::mutex> lock(sim_mtx);
    mj_step(m, d);
}

void Robot::makeContext(mjrContext &con)
{
    mjr_makeContext(m, &con, mjFONTSCALE_150);
}

void Robot::createCamera(mjvCamera &cam, const char *cam_name, int type)
{
    int camID = mj_name2id(m, mjOBJ_CAMERA, cam_name);
    mjv_defaultCamera(&cam);
    cam.fixedcamid = camID;
    cam.type = type;
}

void Robot::updateCamera(mjrRect viewport, mjrContext &con, mjvCamera &c)
{
    {
        std::lock_guard<std::mutex> lock(sim_mtx);
        mjv_updateCamera(m, d, &c, &scn);
    }
    mjr_render(viewport, &scn, &con);
}

void Robot::updateScene(mjrRect viewport, mjrContext &con)
{
    {
        std::lock_guard<std::mutex> lock(sim_mtx);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    }
    mjr_render(viewport, &scn, &con);
}

cv::Matx33d Robot::intrinsic(const char *name, const mjrRect& camView)
{
    int camID = mj_name2id(m, mjOBJ_CAMERA, name);
    
    double fovy = m->cam_fovy[camID];
    double cx = camView.width / 2;
    double cy = camView.height / 2;
    double fy = cy / std::tan((fovy / 2) * M_PI / 180.0);
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);

    K.at<double>(0, 0) = fy;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1.0;

    return K;
}

void Robot::getCamPose(cv::Matx33d& cam_mat,cv::Vec3d& cam_pos,const char* name)
{
    int camID = mj_name2id(m, mjOBJ_CAMERA, name);
     {
        std::lock_guard<std::mutex> lock(sim_mtx);
        cam_pos = cv::Mat(3, 1, CV_64F, d->cam_xpos + camID * 3).clone();
        cam_mat = cv::Mat(3, 3, CV_64F, d->cam_xmat + camID * 9).clone();
    }
    
}