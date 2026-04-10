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

    robot = std::make_unique<Robot>(modelPath);
    window = std::make_unique<Window>();

    mjrContext mainCon;
    mjvCamera cam;

    glfwMakeContextCurrent(window->getWindow());
    robot->makeContext(mainCon);

    robot->createCamera(cam, "endCamera");

    window->setWindowUserPointer(this);
    window->registerCallbacks();

    const mjrRect camview = {0, 0, 640, 480};

    unsigned char *rgbBuffer = new unsigned char[camview.width * camview.height * 3];
    float *depthBuffer = new float[camview.width * camview.height];

    std::thread physic_thread(&Simulate::physicThread, this);

    while (!window->shouldClose())
    {

        glfwMakeContextCurrent(window->getWindow());
        mjrRect viewport = {0, 0, 0, 0};
        window->getFramebufferSize(&viewport.width, &viewport.height);

        robot->updateScene(viewport, mainCon);
        glfwSwapBuffers(window->getWindow());

        robot->updateCamera(camview, mainCon, cam);

        mjr_readPixels(rgbBuffer, depthBuffer, camview, &mainCon);
        cv::Mat image(camview.height, camview.width, CV_8UC3, rgbBuffer);

        cv::flip(image, image, 0);

        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::imshow("CamView", image);
        cv::waitKey(1);

        glfwPollEvents();
    }

    delete[] rgbBuffer;
    delete[] depthBuffer;
    physic_thread.join();
}

void Simulate::physicThread()
{
    while (!window->shouldClose())
    {
    
        auto step_start = std::chrono::high_resolution_clock::now();

        robot->step();
        
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_sec =
            std::chrono::duration<double>(current_time - step_start).count();

        double time_until_next_step = TIMESTEP - elapsed_sec;
        if (time_until_next_step > 0.0)
        {
            auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
            std::this_thread::sleep_for(sleep_duration);
        }
    }
}