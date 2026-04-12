#pragma once
#include <mujoco/mujoco.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>

void scanPluginLibraries();
std::string getExecutableDir();

class Robot
{

private:
    mjvCamera cam;

    mjvOption opt;
    mjvScene scn;

    std::mutex sim_mtx;

    mjModel *m = NULL;
    mjData *d = NULL;

    const double Kp = 50;
    
    cv::Vec3d target_v{0, 0, 0};
    cv::Vec3d target_w{0, 0, 0};
    int cam_id;

    int nv;
    cv::Mat Jp, Jr, J;

public:
    Robot(const char *path)
    {
        scanPluginLibraries();

        char error[1000] = "Could not load binary model";
        m = mj_loadXML(path, nullptr, error, 1000);
        if (!m)
            mju_error("failed to load model: %s", error);

        d = mj_makeData(m);

        int endCameraID = mj_name2id(m, mjOBJ_CAMERA, "endCamera");

        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);

        mjv_makeScene(m, &scn, 2000);

        nv = m->nv;
        Jp = cv::Mat::zeros(3, nv, CV_64F);
        Jr = cv::Mat::zeros(3, nv, CV_64F);
    }

    ~Robot()
    {
        mjv_freeScene(&scn);
        mj_deleteData(d);
        mj_deleteModel(m);
    }

    const mjData *getDataHandle()
    {
        return d;
    }

    void resetRobot();
    void step();
    void updateScene(mjrRect viewport, mjrContext &con);

    void updateCamera(mjrRect viewport, mjrContext &con, mjvCamera &c);

    void createCamera(mjvCamera &cam, const char *cam_name, int type = mjCAMERA_FIXED);

    void moveCamera(int action, mjtNum reldx, mjtNum reldy);
    void makeContext(mjrContext &con);
    cv::Matx33d intrinsic(const char *name, const mjrRect &camView);
    void getCamPose(cv::Matx33d &cam_mat, cv::Vec3d &cam_pos, const char *name);

    void ctrl(const cv::Vec3d &target_v, const cv::Vec3d &target_w, const char *name);
};