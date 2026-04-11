#include "simulate.h"

int main(int argc, const char **argv)
{
    using namespace cv;
    int hmin = 0, smin = 43, vmin = 46;
    int hmax = 20, smax = 255, vmax = 255;

    int kMaxCorners = 4;
    double kQualityLevel = 0.1;
    double kMinDistance = 1;

    Simulate simulate(argv[1]);
    Mat image, imageHSV, mask;

    while (1)
    {
        simulate.getImage(image);
        cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

        imshow("Camera View", image);
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imageHSV, lower, upper, mask);

        std::vector<Point2f> corners;
        goodFeaturesToTrack(mask, corners, kMaxCorners, kQualityLevel, kMinDistance);

                
        for (size_t i = 0; i < corners.size(); i++)
            circle(image, corners[i], 2.5, Scalar(255, 0, 0), 1);
        
        imshow("corner Image", image);
        waitKey(1);

        TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
        cornerSubPix(mask, corners, Size(5, 5), Size(-1, -1), criteria);
    }
    return 0;
}
