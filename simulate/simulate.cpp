#include "simulate.h"

void Window::mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{

    Simulate *sim = static_cast<Simulate *>(glfwGetWindowUserPointer(window));
    if (sim)
        sim->window->handle_mouse_button();
}

void Window::keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    Simulate *sim = static_cast<Simulate *>(glfwGetWindowUserPointer(window));
    if (sim)
    {
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
            sim->robot->resetRobot();
    }
}

void Window::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
    Simulate *sim = static_cast<Simulate *>(glfwGetWindowUserPointer(window));
    if (sim)
    {
        int action;
        double reldx, reldy;

        if (sim->window->handle_mouse_move(xpos, ypos, action, reldx, reldy))
            sim->robot->moveCamera(action, reldx, reldy);
    }
}

void Window::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    Simulate *sim = static_cast<Simulate *>(glfwGetWindowUserPointer(window));

    if (sim)
        sim->robot->moveCamera(mjMOUSE_ZOOM, 0, -0.05 * yoffset);
}

void Simulate::simThread()
{

    window = std::make_unique<Window>();

    mjrContext mainCon;
    mjr_defaultContext(&mainCon);

    robot->makeContext(mainCon);

    window->setWindowUserPointer(this);
    window->registerCallbacks();

    physic_thread = std::thread(&Simulate::physicThread, this);

    while (!window->shouldClose())
    {
        mjrRect viewport = {0, 0, 0, 0};
        window->getFramebufferSize(&viewport.width, &viewport.height);

        robot->updateScene(viewport, mainCon);
        glfwSwapBuffers(window->getWindow());

        robot->updateCamera(camview, mainCon, cam);

        {
            std::unique_lock<std::mutex> lock(buffer_mtx);
            if (!doubleBuffer.ready)
            {
                mjr_readPixels(doubleBuffer.getWriteBuffer(), NULL, camview, &mainCon);
                doubleBuffer.ready = true;
                lock.unlock();
                cr.notify_all();
            }
        }

        glfwPollEvents();
    }
}

void Simulate::physicThread()
{
    const mjData *d = robot->getDataHandle();

    time_point cpu_start = Clock::now();
    mjtNum sim_start = d->time;

    while (!window->shouldClose())
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        while (d->time - sim_start < std::chrono::duration<double>(Clock::now() - cpu_start).count())
        {
            robot->step();
        }
    }
}

void Simulate::ctrl(const cv::Vec3d &target_v, const cv::Vec3d &target_w, const char *name)
{
    this->robot->ctrl(target_v, target_w, name);
}