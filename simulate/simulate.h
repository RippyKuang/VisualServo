#include "robot.h"
#include "window.h"
#include <memory>
#include <mujoco/mujoco.h>

class Simulate
{
private:
    void simThread();
    std::thread sim_thread;
    const char *modelPath;

public:
    std::unique_ptr<Robot> robot;
    std::unique_ptr<Window> window;

    Simulate(const char *path) : modelPath(path)
    {
        sim_thread = std::thread(&Simulate::simThread, this);
    }
    ~Simulate()
    {
        sim_thread.join();
    }
};