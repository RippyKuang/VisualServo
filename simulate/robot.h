#pragma once
#include <mujoco/mujoco.h>
#include <cstring>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

void scanPluginLibraries();
std::string getExecutableDir();

class Robot
{
private:
    mjModel *m = NULL;
    mjData *d = NULL;
    mjvCamera cam;

    mjvOption opt;
    mjvScene scn;

public:
    Robot(const char *path)
    {
        scanPluginLibraries();

        char error[1000] = "Could not load binary model";
        m = mj_loadXML(path, nullptr, error, 1000);
        if (!m)
            mju_error("加载模型失败: %s", error);

        d = mj_makeData(m);

        int endCameraID = mj_name2id(m, mjOBJ_CAMERA, "endCamera");
        if (endCameraID == -1)
            std::cerr << "Camera not found" << std::endl;

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

    void resetRobot();
    void step();
    void updateScene(mjrRect viewport, mjrContext &con)
    {
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
    }
    void updateCamera(mjrRect viewport, mjrContext &con, mjvCamera &c)
    {
        mjv_updateCamera(m, d, &c, &scn);
        mjr_render(viewport, &scn, &con);
    }
    void createCamera(mjvCamera &cam, const char *cam_name, int type = mjCAMERA_FIXED)
    {
        int camID = mj_name2id(m, mjOBJ_CAMERA, cam_name);
        mjv_defaultCamera(&cam);
        cam.fixedcamid = camID;
        cam.type = type;
    }
    void moveCamera(int action, mjtNum reldx, mjtNum reldy);
    void makeContext(mjrContext &con);
};