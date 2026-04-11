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