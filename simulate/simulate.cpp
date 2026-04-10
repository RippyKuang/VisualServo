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

    robot = std::make_unique<Robot>(modelPath);
    window = std::make_unique<Window>(); 


    GLFWwindow* camView = glfwCreateWindow(640, 480, "End Effector Camera", NULL, NULL);
  

    mjrContext mainCon, camCon;
    mjvCamera cam; 

    
    glfwMakeContextCurrent(window->getWindow());
    robot->makeContext(mainCon); 

    
    glfwMakeContextCurrent(camView);
    robot->makeContext(camCon);
    robot->createCamera(cam, "endCamera"); 

    window->setWindowUserPointer(this);
    window->registerCallbacks();

    while (!window->shouldClose() && !glfwWindowShouldClose(camView))
    {
        
        robot->step();

        glfwMakeContextCurrent(window->getWindow()); 
        mjrRect viewport = {0, 0, 0, 0};
        window->getFramebufferSize(&viewport.width, &viewport.height);
      
        robot->updateScene(viewport, mainCon); 
        glfwSwapBuffers(window->getWindow());

     
        glfwMakeContextCurrent(camView); 
        glfwGetFramebufferSize(camView, &viewport.width, &viewport.height);
        
     
        robot->updateCamera(viewport, camCon,cam); 
        glfwSwapBuffers(camView);

        glfwPollEvents();
        
      
    }
}

