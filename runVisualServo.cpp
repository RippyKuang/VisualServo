#include "simulate.h"
#include "imgProcess.h"

int main(int argc, const char **argv)
{
    using namespace cv;

    Simulate simulate(argv[1]);
    Mat image;

    std::vector<Point2f> corners;
    corners.reserve(kMaxCorners);

    Mat K = simulate.intrinsic("endCamera");

    while (1)
    {
        simulate.getImage(image);

        if (findKeyPoints(image, corners, true))
        {
           
        }
        corners.clear();
    }
    return 0;
}
