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

void Robot::makeContext(mjrContext& con)
{
    mjr_makeContext(m, &con, mjFONTSCALE_150);
}