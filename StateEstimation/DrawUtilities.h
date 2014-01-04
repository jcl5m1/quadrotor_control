#ifndef DRAWUTILITIES_H
#define DRAWUTILITIES_H

#include "Eigen/Core"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace Eigen;

inline double Gaussian(double x, double mu, double sigma) {
    double e = (x-mu)/sigma;
    return exp(-0.5*e*e)/(sigma*sqrt(2*M_PI));
}

inline void DrawCrossHair(Mat img, const Point2f & p, const int size, const Scalar color) {
    line(img, Point(p.x - size, p.y), Point(p.x + size, p.y),color);
    line(img, Point(p.x, p.y - size), Point(p.x, p.y + size),color);
}

inline void DrawCrossHair(Mat img, const Point & p, const int size,const Scalar color) {
    line(img, Point(p.x - size, p.y), Point(p.x + size, p.y),color);
    line(img, Point(p.x, p.y - size), Point(p.x, p.y + size),color);
}

inline void DrawGaussian(Mat img, Vector2f g, Scalar color, float scale) {

    Point p1;
    Point p2;
    int step = 10;
    for(int x = 0; x < img.cols-step; x++) {
        p1.x = step*x;
        p2.x = step*(x+1);
        p1.y = img.rows - scale*Gaussian(x,g[0], g[1])-20;
        p2.y = img.rows - scale*Gaussian(x+1,g[0], g[1])-20;
        line(img,p1,p2,color);
    }
}


#endif // DRAWUTILITIES_H
