#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "Eigen/Core"
#include "Eigen/LU"

using namespace Eigen;
using namespace std;

template <class T>
inline void printMatrix(const char * name,const T & m) {
    cout << name << "----\n" << m << endl;
}


class KalmanFilter {

public:
    KalmanFilter(){}
    Vector2f measurementUpdate1D(Vector2f s, Vector2f m) {
        s[0] = (s[0]*m[1] + m[0]*s[1])/(s[1] + m[1]); //compute updated state from predicted and measured
        s[1] = 1/(1/s[1] + 1/m[1]); //combine current uncertaintly with measurement uncertainty
        return s;
    }

    Vector2f predictionUpdate1D(Vector2f s, Vector2f m) {
        s[0] += m[0]; //update new state
        s[1] += m[1]; //update new uncertainty
        return s;
    }

    void MultiDimensionalTest() {
        Matrix<float, 2,1> x; x << 0,0; //state estimate pos & velocity
        Matrix<float, 2,2> P; P << 1000, 0,  0,  1000; //uncertainty covariance
        Matrix<float, 2,1> u; u << 0,0; //motion change
        Matrix<float, 2,2> F; F << 1,1,0,1; //state transition matrix
        Matrix<float, 1,2> H; H << 1,0; //measurement function
        Matrix<float, 1,1> R; R << 1; //measurement noise
        Matrix<float, 2,2> I; I << 1,0,0,1; //identity


        const int count = 3;
        float measurement[count] = {1,2,3}; //measurements

        Matrix<float, 1,1> y;
        Matrix<float, 1,1> z;
        Matrix<float, 1,1> S;
        Matrix<float, 2,1> K;

        for(int i = 0; i < count; i++) {
            //measurement
            z[0] = measurement[i];

            y = z - H*x; //error between predicted and measured values
            S = H*P*H.transpose() + R;//compute uncertainy of measurement given our current uncertainty
            K = P*H.transpose() * S.inverse();//compute scaling coefficient for the correction
            x = x + (K*y);//correct our state based off a scaled version of the error
            P = (I - K*H)*P;//compute updated uncertainty covariance

            //prediction
            x = F*x + u;//update state using transition matrix plus external impact
            P = F*P*F.transpose(); //update the covariance using the transition matrix

            printMatrix("x",x);
            printMatrix("P",P);
        }
    }


    void MultiDimensionalTest2() {

        //this a 1D position but with hidden velocity and acceleration
        const int stateSize = 3;
        float initialVariance = 1000;
        Matrix<float, stateSize,1> x; //state estimate
        Matrix<float, stateSize,stateSize> P; //uncertainty covariance
        P  = MatrixXf::Identity(stateSize,stateSize)*initialVariance;
        Matrix<float, stateSize,1> u; //motion change
        Matrix<float, stateSize,stateSize> F;//state transition matrix
        Matrix<float, 1,stateSize> H; //measurement function
        Matrix<float, 1,1> R; R << 1; //measurement noise
        Matrix<float, stateSize,stateSize> I; //identity
        I = MatrixXf::Identity(stateSize,stateSize);


        x << 0,0,0; //state estimate x,y pos and velocity
        //state transition matrix
        F << 1,1,0.5, //pos' = pos + vel + 0.5*accel
             0,1,1, //vel' = vel + accel
             0,0,1;//accel' = accel
        //measurement function
        H << 1,0,0; //only measure position
        //motion change
        u << 0,0,0;//nothing outside of state estimate


        //position measurements
        const int count = 9;
        float measurement[count] = {
        0,
        4.905,
        19.62,
        44.145,
        78.48,
        122.625,
        176.58,
        240.345,
        313.92};

        Matrix<float, 1,1> y;
        Matrix<float, 1,1> z;
        Matrix<float, 1,1> S;
        Matrix<float, stateSize,1> K;

        for(int i = 0; i < count; i++) {
            //measurement
            z[0] = measurement[i];
            printMatrix("measured",z);

            printMatrix("predicted",x);
            printMatrix("covariance",P);

            y = z - H*x; //error between predicted and measured values
            S = H*P*H.transpose() + R;//project system uncertainty into the measurement space with noise
            K = P*H.transpose() * S.inverse();//compute scaling coefficient for the correction
            x = x + (K*y);//correct our state based off a scaled version of the error
            P = (I - K*H)*P;//compute updated uncertainty covariance

            //prediction
            x = F*x + u;//update state using transition matrix plus external impact
            P = F*P*F.transpose(); //update the covariance using the transition matrix

            printMatrix("updated prediction",x);

            printf("************\n");
        }
    }


    void PosVelAccel2DTest() {

        //this a 2D position with hidden velocity and acceleration on each axis
        const int stateSize = 6;
        const int measureDim = 2;
        float initialVariance = 1000;
        Matrix<float, stateSize,1> x; //state estimate
        Matrix<float, stateSize,stateSize> P; //uncertainty covariance
        P  = MatrixXf::Identity(stateSize,stateSize)*initialVariance;
        Matrix<float, stateSize,1> u; //motion change
        Matrix<float, stateSize,stateSize> F;//state transition matrix
        Matrix<float, measureDim,stateSize> H; //measurement function
        Matrix<float, measureDim,measureDim> R; //measurement noise
        Matrix<float, stateSize,stateSize> I; //identity
        I = MatrixXf::Identity(stateSize,stateSize);


        x << 0,0,0,0,0,0; //state estimate x,y pos and velocity
        //state transition matrix
        F << 1,1,0.5,   0,0,0, //pos' = pos + vel + 0.5*accel
             0,1,1,     0,0,0, //vel' = vel + accel
             0,0,1,     0,0,0,
             0,0,0,     1,1,0.5,
             0,0,0,     0,1,1,
             0,0,0,     0,0,1;//accel' = accel
        //measurement function
        H << 1,0,0,0,0,0,
             0,0,0,1,0,0; //only measure positions
        //motion change
        u << 0,0,0,0,0,0;//nothing outside of state estimate

        //measurement noise
        R << 1,0, //x is 1
             0,1; //y is 1


        //position measurements
        const int measurementCount = 4;
        Matrix<float,measurementCount,measureDim> measurement;
        measurement <<
             5,             0,
           9.9,             1,
           24.62,         4,
           49.145,        9;

        Matrix<float, measureDim,1> y;
        Matrix<float, measureDim,1> z;
        Matrix<float, measureDim,measureDim> S;
        Matrix<float, stateSize,measureDim> K;

        for(int i = 0; i < measurementCount; i++) {
            //measurement
            z = measurement.row(i);
            printMatrix("measured",z);

            printMatrix("predicted",x);
            printMatrix("covariance",P);

            y = z - H*x; //error between predicted and measured values
            S = H*P*H.transpose() + R;//project system uncertainty into the measurement space with noise
            K = P*H.transpose() * S.inverse();//compute scaling coefficient for the correction
            x = x + K*y;//correct our state based off a scaled version of the error
            P = (I - K*H)*P;//compute updated uncertainty covariance

            //prediction
            x = F*x + u;//update state using transition matrix plus external impact
            P = F*P*F.transpose(); //update the covariance using the transition matrix

            printMatrix("updated prediction",x);

            printf("************\n");
        }
    }

};



class KalmanFilterPosVelAccel2D {

    static const int xPos = 0;
    static const int xVel = 1;
    static const int xAcc = 2;
    static const int yPos = 3;
    static const int yVel = 4;
    static const int yAcc = 5;

    static const int stateSize = 6;
    static const int measureDim = 2;

    static const int historySize = 10;

    float initialVariance;
    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance
    Matrix<float, stateSize,1> u; //motion change
    Matrix<float, stateSize,stateSize> F;//state transition matrix
    Matrix<float, measureDim,stateSize> H; //measurement function
    Matrix<float, measureDim,measureDim> R; //measurement noise
    Matrix<float, stateSize,stateSize> I; //identity


    Matrix<float, measureDim,1> y;
    Matrix<float, measureDim,1> z;
    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;

    Matrix<float, historySize, measureDim> measurementHistory;
    int historyIndex;

public:
    KalmanFilterPosVelAccel2D(float initVariance = 1000){

        //variance of all state varables, same for all
        initialVariance = initVariance;

        historyIndex = 0;

        ResetEstimation();

        //motion change
        u << 0,0,0,0,0,0;//nothing outside of state estimate

        //measurement noise - default to identity
        R =  MatrixXf::Identity(measureDim, measureDim);

        //state transition matrix
        F << 1,1,0.5,   0,0,0, //pos' = pos + vel + 0.5*accel
             0,1,1,     0,0,0, //vel' = vel + accel
             0,0,1,     0,0,0, //accel' = accel
             0,0,0,     1,1,0.5,
             0,0,0,     0,1,1,
             0,0,0,     0,0,1;
        //measurement function
        H << 1,0,0,0,0,0,
             0,0,0,1,0,0; //only measure positions

        I = MatrixXf::Identity(stateSize,stateSize);
    }


    Vector2f PredictedPos() {
        Vector2f result(x(xPos),x(yPos));
        return result;
    }

    Matrix<float, stateSize,stateSize> Covariance() {
        return P;
    }

    void ResetEstimation() {
        //state estimate x_pos, x_vel, x_acc, y_pos, y_vel, y_acc
        x << 0,0,0,0,0,0;
        P  = MatrixXf::Identity(stateSize,stateSize)*initialVariance;
    }

    void UdpateUsingWindow(Matrix<float, measureDim,1> z, float measurementVar = 1) {

        for(int i = 0; i < measureDim; i++) {
            measurementHistory(historyIndex%historySize,i) = z(i);
        }

        int start = historyIndex - historySize+1;
        if(start < 0)
            start = 0;

        ResetEstimation();
        for(int i = start; i <= historyIndex; i++)
            Update(measurementHistory.row(i%historySize),measurementVar);

        historyIndex++;

    }

    void Update(Matrix<float, measureDim,1> z, float measurementVar = 1) {

        //measurement noise - made up
        R << measurementVar,0, //x is 1
             0,measurementVar; //y is 1

        //measurement update
        y = z - H*x; //error between predicted and measured values
        S = H*P*H.transpose() + R;//project system uncertainty into the measurement space with noise
        K = P*H.transpose() * S.inverse();//compute scaling coefficient for the correction
        x = x + K*y;//correct our state based off a scaled version of the error
        P = (I - K*H)*P;//compute updated uncertainty covariance

        //update prediction
        x = F*x + u;//update state using transition matrix plus external impact
        P = F*P*F.transpose(); //update the covariance using the transition matrix
    }

    void Test() {

        //position measurements
        const int measurementCount = 5;
        Matrix<float,measurementCount,measureDim> measurement;
        measurement <<
           5, 0,
           7, 1,
           13, 4,
           23, 9,
           37, 16;

        for(int i = 0; i < measurementCount; i++) {
            Update(measurement.row(i));
        }

        Matrix<float, stateSize,1> ans;
        ans <<  54.993,
                19.9933,
                3.99743,
                24.998,
                9.99829,
                1.99943;

        float diff = (ans-x).squaredNorm();
        if(diff > 0.001) {
            printf("FAILED TEST with delta %f\n", diff);
        } else {
            printf("PASSED TEST\n");
        }
    }

};

#endif // KALMANFILTER_H
