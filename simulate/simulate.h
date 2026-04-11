#include "robot.h"
#include "window.h"
#include <memory>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <mujoco/mujoco.h>

#define TIMESTEP 0.002

typedef std::chrono::_V2::system_clock::time_point time_point;
typedef std::chrono::high_resolution_clock Clock;

struct Buffer
{
    uint8_t *buffers[2];
    int idx = 0;
    bool ready = false;

    Buffer(int size)
    {
        buffers[0] = new uint8_t[size];
        buffers[1] = new uint8_t[size];
    }

    ~Buffer()
    {
        delete[] buffers[0];
        delete[] buffers[1];
    }

    uint8_t *getWriteBuffer()
    {
        return buffers[idx];
    }

    uint8_t *getReadBuffer()
    {
        return buffers[1 - idx];
    }

    void swap()
    {
        idx = 1 - idx;
    }
};
class Simulate
{
    friend class robot;
    friend class Window;

private:
    void simThread();
    void physicThread();

    std::thread sim_thread;
    std::thread physic_thread;

    const char *modelPath;
    mjvCamera cam;
    const mjrRect camview = {0, 0, 640, 480};

    std::mutex buffer_mtx;
    std::condition_variable cr;
    bool frame_ready = false;

    Buffer doubleBuffer{camview.width * camview.height * 3};

protected:
    std::unique_ptr<Robot> robot;
    std::unique_ptr<Window> window;

public:
    Simulate(const char *path) : modelPath(path)
    {

        robot = std::make_unique<Robot>(modelPath);
        robot->createCamera(cam, "endCamera");

        sim_thread = std::thread(&Simulate::simThread, this);
    }

    ~Simulate()
    {
        sim_thread.join();
        physic_thread.join();
    }

    void getImage(cv::Mat &m)
    {

        std::unique_lock<std::mutex> lock(buffer_mtx);
        cr.wait(lock, [&]
                { return doubleBuffer.ready; });

        doubleBuffer.swap();
        uint8_t *buffer = doubleBuffer.getReadBuffer();
        cv::Mat image(camview.height, camview.width, CV_8UC3, buffer);
        doubleBuffer.ready = false;
        lock.unlock();

        image.copyTo(m);
        cv::flip(m, m, 0);
        cv::cvtColor(m, m, cv::COLOR_BGR2RGB);
    }
    
    cv::Mat intrinsic(const char *name)
    {
        return robot->intrinsic(name, camview);
    }
};