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
    cv::Mat intrinsic(const char* name,const mjrRect& camView);
    void getCamPose(cv::Mat& cam_mat,cv::Mat& cam_pos,const char* name);
};