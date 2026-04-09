#include "window.h"

Window::~Window()
{
    glfwDestroyWindow(window);
    glfwTerminate();
}

bool Window::shouldClose()
{
    return glfwWindowShouldClose(window);
}

void Window::setWindowUserPointer(void *p)
{
    glfwSetWindowUserPointer(window, p);
}

void Window::handle_mouse_button()
{
    // update button state
    button_left =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

bool Window::handle_mouse_move(double xpos, double ypos, int &action, double &reldx, double &reldy)
{

    if (!button_left && !button_middle && !button_right)
        return false;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    reldx = dx / height;
    reldy = dy / height;

    if (button_right)
    {
        action = mod_shift ? MOUSE_MOVE_H : MOUSE_MOVE_V;
    }
    else if (button_left)
    {
        action = mod_shift ? MOUSE_ROTATE_H : MOUSE_ROTATE_V;
    }
    else
    {
        action = MOUSE_ZOOM;
    }

    return true;
}