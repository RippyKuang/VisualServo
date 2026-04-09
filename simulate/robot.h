#include <mujoco/mujoco.h>
#include <cstring>
#include <iostream>
#include <thread>

#define MUJOCO_PLUGIN_DIR "/usr/local/bin/mujoco_plugin"


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
    mjrContext con;

public:
    Robot(const char *path)
    {
        scanPluginLibraries();

        char error[1000] = "Could not load binary model";
        m = mj_loadXML(path, nullptr, error, 1000);
        if (!m)
            mju_error("加载模型失败: %s", error);

        d = mj_makeData(m);

        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);
    }

    ~Robot()
    {

        mjv_freeScene(&scn);
        mjr_freeContext(&con);

        mj_deleteData(d);
        mj_deleteModel(m);
    }

    void resetRobot();
    void step();
    void update(mjrRect viewport)
    {
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
    }
    void moveCamera(int action, mjtNum reldx, mjtNum reldy);
};