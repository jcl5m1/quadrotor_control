#include <iostream>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Eigen/Core"

#include "KalmanFilter.h"
#include "DrawUtilities.h"

using namespace std;
using namespace cv;
using namespace Eigen;

Point2f mousePos(0,0);
//KalmanFilterPosVelAccel2D kf(10000);
KalmanFilterPosVel2D kf(10000);

void onMouse(int event, int x, int y, int flags, void * data){
    mousePos.x = x;
    mousePos.y = y;
}

int main(int argc, char** argv)
{
    printf("StateEstimation\n");

    Mat img(512,512,CV_8UC3, Scalar(0,0,0));
    char * windowName = "Image";
    namedWindow(windowName);

    setMouseCallback(windowName, onMouse);

    kf.Test();

    kf.ResetEstimation();

    while(1) {
        img.setTo(Scalar(0,0,0));
        DrawCrossHair(img, mousePos,5);

        Vector2f meas(mousePos.x,mousePos.y);
        kf.Update(meas,10);
        printMatrix("covar", kf.Covariance());

        Vector2f pos = kf.PredictedPos();
        ellipse(img, Point(pos[0], pos[1]),Size(20,20),0,0,360,Scalar(0,255,0));
        imshow(windowName, img);
        char c = waitKey(30);
        if(c == 27)
            break;
        if(c == 'r')
            kf.ResetEstimation();
    }

    return 0;
}
