#include <GLFW/glfw3.h>


#define MOUSE_MOVE_H 4
#define MOUSE_MOVE_V 3
#define MOUSE_ROTATE_H 2
#define MOUSE_ROTATE_V 1
#define MOUSE_ZOOM 5

class Window
{
private:
    GLFWwindow *window;
    double lastx = 0;
    double lasty = 0;
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;

    void handle_mouse_button();
    bool handle_mouse_move(double xpos, double ypos, int &action, double &reldx, double &reldy);
    static void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
    static void keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
    static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

    
public:
    Window()
    {
        glfwInit();
        window = glfwCreateWindow(1920, 1440, "Demo", NULL, NULL);
        glfwMakeContextCurrent(window);
      //  glfwSwapInterval(1);
    }
    ~Window();

    bool shouldClose();
    void setWindowUserPointer(void *p);

    void getFramebufferSize(int *width, int *height)
    {
        glfwGetFramebufferSize(window, width, height);
    }
    GLFWwindow* getWindow()const
    {
        return window;
    }


    void registerCallbacks()
    {
        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetKeyCallback(window, keyboard_callback);
        glfwSetCursorPosCallback(window, mouse_move_callback);
        glfwSetScrollCallback(window, scroll_callback);
    }
};