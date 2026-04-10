#include "robot.h"
#include "window.h"
#include <memory>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <mujoco/mujoco.h>

#define TIMESTEP 0.002

class Simulate
{
private:
    void simThread();
    void physicThread();

    std::thread sim_thread;
    std::thread physic_thread;

    const char *modelPath;
    const mjrRect camview = {0, 0, 640, 480};
    
    std::mutex buffer_mtx;
    std::condition_variable cr;
    bool frame_ready = false;

    uint8_t *imgBuffer;

public:
    std::unique_ptr<Robot> robot;
    std::unique_ptr<Window> window;

    Simulate(const char *path) : modelPath(path)
    {
        sim_thread = std::thread(&Simulate::simThread, this);
        imgBuffer = new uint8_t[camview.width * camview.height * 3];
    }

    ~Simulate()
    {
        sim_thread.join();
        physic_thread.join();

        delete[] imgBuffer;
    }

    void getImage(cv::Mat &m)
    {
        std::unique_lock<std::mutex> lock(buffer_mtx);
        cr.wait(lock, [&]
                { return frame_ready; });

        cv::Mat image(camview.height, camview.width, CV_8UC3, imgBuffer);
        frame_ready = false;
        image.copyTo(m);
        lock.unlock();
        cv::flip(m, m, 0);
        cv::cvtColor(m, m, cv::COLOR_BGR2RGB);
    }
};