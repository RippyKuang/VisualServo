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
        {
            sim->robot->resetRobot();
        }
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
    {
        sim->robot->moveCamera(mjMOUSE_ZOOM, 0, -0.05 * yoffset);
    }
}

void Simulate::simThread()
{

    window = std::make_unique<Window>();
    robot = std::make_unique<Robot>(modelPath);

    window->setWindowUserPointer(this);
    window->registerCallbacks();

    auto step_start = std::chrono::high_resolution_clock::now();
    while (!window->shouldClose())
    {

        robot->step();

        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_sec =
            std::chrono::duration<double>(current_time - step_start).count();
        double time_until_next_step = 0.002 * 5 - elapsed_sec;
        if (time_until_next_step > 0.0)
        {
            auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
            std::this_thread::sleep_for(sleep_duration);
        }

        mjrRect viewport = {0, 0, 0, 0};
        window->getFramebufferSize(&viewport.width, &viewport.height);

        robot->update(viewport);

        glfwSwapBuffers(window->getWindow());
        glfwPollEvents();
    }
}