#include "simulate.h"
#include "imgProcess.h"

int main(int argc, const char **argv)
{
    using namespace cv;


    Simulate simulate(argv[1]);
    Mat image, imageHSV, mask;
    std::vector<Point2f> corners;
    corners.reserve(kMaxCorners);
    while (1)
    {
        simulate.getImage(image);
        getCornerSubPix(image,corners);
        corners.clear();
    }
    return 0;
}
