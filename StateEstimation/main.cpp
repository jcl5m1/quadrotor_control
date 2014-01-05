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
KalmanFilterNinjaQuail kf;

void onMouse(int event, int x, int y, int flags, void * data){
    mousePos.x = x;
    mousePos.y = y;
}

class Patricle {
public:

    Matrix<float,6,1> state;
    Matrix<float,6,6> updateFunction;
    float dt;
    float drag;

    void Init(float x, float y, float v) {
        dt = 0.1;
        //x' = x + v*dt + a*0.5*dt*dt
        //v' = v + a*dt
        //a' = a

        drag = 0.995;

        updateFunction <<
        1,dt,0.5*dt*dt, 0,0,0,
        0,drag,dt,         0,0,0,
        0,0,1,          0,0,0,

        0,0,0,        1,dt,0.5*dt*dt,
        0,0,0,        0,drag,dt,
        0,0,0,        0,0,1;

        setPos(x,y);

        float m = randf();
        state(1) = v*cos(2*M_PI*m);
        state(4) = v*sin(2*M_PI*m);
    }

    void ComputeGravity(float x, float y, float g) {
        float dx = state(0) - x;
        float dy = state(3) - y;

        float mag = sqrt(dx*dx + dy*dy);

        dx /= mag;
        dy /= mag;

        state(2) = -dx*g/(mag + 10);
        state(5) = -dy*g/(mag + 10);
    }


    void setPos(float x, float y) {
        state(0) = x;
        state(3) = y;
    }


    Point getPos() {
        Point p;
        p.x = state(0);
        p.y = state(3);
        return p;
    }

    Point getAccel() {
        Point p;
        p.x = state(2);
        p.y = state(5);
        return p;
    }


    void Update() {
        state = updateFunction*state;
    }
};

const int particleCount = 5;
Patricle particles[particleCount];
int width = 800;
int height = 800;

int main(int argc, char** argv)
{
    printf("StateEstimation\n");

    Mat img(width, height,CV_8UC3, Scalar(0,0,0));
    char * windowName = "Image";
    namedWindow(windowName);

    setMouseCallback(windowName, onMouse);

    const int count = (int)(kf.duration/kf.dt);
    float Qpos[count];
    float Qvel[count];
    float Qpos_meas[count];
    float Qpos_kf[count];
    float Qprepos_kf[count];

    Matrix<float, 1,1> u;
    Matrix<float, 1,1> z;

    u << 0.0;
    float meas_noise = 10;

    Qpos[0] = 0.0f;
    Qvel[0] = 0.0f;
    Qpos_meas[0] = 0.0f;
    Qpos_kf[0] = kf.State()(0);
    Qprepos_kf[0] = kf.PredictedState()(0);

    Matrix<float, KalmanFilterNinjaQuail::measureDim, KalmanFilterNinjaQuail::measureDim> Ez;
    Ez << meas_noise;

    kf.ResetEstimation();
    for(int i = 1; i < count+1; i++) {
        printf("interation %d *****************\n", i);
        Qvel[i] = 16;// + u(0)*kf.dt;
        Qpos[i] = Qpos[i-1] + Qvel[i]*kf.dt;
        Qpos_meas[i] = Qpos[i] + meas_noise*randfGaussian();
        Qpos_kf[i] = kf.State()(0);
        z << Qpos_meas[i];
        kf.Update(z,Ez,u);
        Qpos_kf[i] = kf.State()(0);
        Qprepos_kf[i] = kf.PredictedState()(0);
    }

    for(int i = 0; i < particleCount; i++) {
        particles[i].Init(width*randf(),height*randf(),50);
//        particles[i].ComputeGravity(width/2, height/2);
    }

    while(1) {
        printf("*************\n");
        img.setTo(Scalar(0,0,0));
        DrawCrossHair(img, mousePos,5);

        int xStep = 3;
        for(int i = 0; i < count-1; i++) {
//            Vector2f pt = randVec2fGaussian(150,250,50,50);
  //          circle(img, Point(pt[0], pt[1]),3,Scalar(0,255,0));


//            line(img,Point(i*xStep, img.rows - Qpos[i]),Point((i+1)*xStep,img.rows -Qpos[i+1]),Scalar(0,0,255));
//            line(img,Point(i*xStep, img.rows - Qpos_meas[i]),Point((i+1)*xStep,img.rows -Qpos_meas[i+1]),Scalar(255,255,255));

//            line(img,Point(i*xStep, img.rows - Qpos_kf[i]),Point((i+1)*xStep,img.rows -Qpos_kf[i+1]),Scalar(0,255,0));
//            line(img,Point(i*xStep, img.rows - Qprepos_kf[i]),Point((i+1)*xStep,img.rows -Qprepos_kf[i+1]),Scalar(255,255,0));

        }
        for(int i = 0; i < particleCount; i++) {
            particles[i].ComputeGravity(width/2, height/2,10000);
            particles[i].Update();
            Point p1 = particles[i].getPos();
            Point p2 = p1 + particles[i].getAccel();
            circle(img, p1,3,Scalar(0,255,255));
        }

        imshow(windowName, img);
        char c = waitKey(30);
        if(c == 27)
            break;
        if(c == 'r')
            kf.ResetEstimation();
    }

    return 0;
}
