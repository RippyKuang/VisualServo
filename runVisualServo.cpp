#include "simulate.h"

int main(int argc, const char **argv)
{
  
    Simulate simulate(argv[1]);
    cv::Mat image;

    while (1)
    {
        simulate.getImage(image);
        cv::imshow("Camera View", image);
        cv::waitKey(1);
    }
    return 0;
}
