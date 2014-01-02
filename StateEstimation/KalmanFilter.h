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


class KalmanFilter1D {

public:
    KalmanFilter1D(){}
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
};


class KalmanFilterGeneral1D {

    static const int stateSize = 1;
    static const int measureDim = 1;

    float initialVariance;
    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance
    Matrix<float, stateSize,1> u; //motion change
    Matrix<float, measureDim,measureDim> R; //measurement noise
    Matrix<float, stateSize,stateSize> I; //identity

    Matrix<float, measureDim,1> y;
    Matrix<float, measureDim,1> z;
    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;

public:
    KalmanFilterGeneral1D(float initVariance = 1000){

        //variance of all state varables, same for all
        initialVariance = initVariance;

        ResetEstimation();

        //motion change
        u << 0;//nothing outside of state estimate

        //measurement noise - default to identity
        R =  MatrixXf::Identity(measureDim, measureDim);

        I = MatrixXf::Identity(stateSize,stateSize);
    }

    void ResetEstimation() {
        //state estimate
        x << 0;
        P  = MatrixXf::Identity(stateSize,stateSize)*initialVariance;
    }

    void Update(Matrix<float, measureDim,1> z, float measurementVar = 1) {


        //need to understand how this simplies in to the above... if it does (which it should)

        //measurement noise - made up
        R << measurementVar;

        //measurement update
        y = z - x; //error between predicted and measured values
        S = P + R;//project system uncertainty into the measurement space with noise
        K = P * S.inverse();//compute scaling coefficient for the correction
        x = x + K*y;//correct our state based off a scaled version of the error
        P = (I - K)*P;//compute updated uncertainty covariance

        //update prediction
        x = x + u;//update state using transition matrix plus external impact
        P = P; //update the covariance using the transition matrix
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

//kalman with motor command
class KalmanFilterPosVel2D {

    static const int xPos = 0;
    static const int xVel = 1;
    static const int yPos = 2;
    static const int yVel = 3;

    static const int stateSize = 4;
    static const int measureDim = 2;

    static const int historySize = 10;

    float initialVariance;
    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance
    Matrix<float, stateSize,stateSize> pP; //predicted uncertainty covariance
    Matrix<float, stateSize,stateSize> A;//state1 to state2 equations
    Matrix<float, stateSize,stateSize> B;//control input to state equations
    Matrix<float, stateSize,1> u; //control input matrix
    Matrix<float, stateSize,stateSize> Ex; //noise of a state prediction
    Matrix<float, stateSize,1> px; //state update noise

    Matrix<float, measureDim,stateSize> C; //state-to-measurement equation
    Matrix<float, measureDim,measureDim> Ez; //measurement prediction noise
    Matrix<float, measureDim,1> pz; //predicted measurement

    Matrix<float, stateSize,stateSize> I; //identity

    Matrix<float, measureDim,1> y;
    Matrix<float, measureDim,1> z;
    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;

    Matrix<float, historySize, measureDim> measurementHistory;
    int historyIndex;

public:
    KalmanFilterPosVel2D(float initVariance = 1000){

        //variance of all state varables, same for all
        initialVariance = initVariance;

        historyIndex = 0;

        ResetEstimation();

        //motion change
        u.setZero();//nothing outside of state estimate

        //measurement noise - default to identity
        Ex =  MatrixXf::Identity(measureDim, measureDim);

        //state transition matrix
        A << 1,1,   0,0, //pos' = pos + vel
             0,1,   0,0, //vel' = vel
             0,0,   1,1,
             0,0,   0,1;
        //measurement function
        C << 1,0,0,0,
             0,0,1,0; //only measure positions

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
        x.setZero();
        P  = MatrixXf::Identity(stateSize,stateSize)*initialVariance;
    }

    void Update(Matrix<float, measureDim,1> z, float measurementVar = 1) {

        //predicted state = transitionMat*state + controlMat*control
        px = A*x + B*u;

        //predictedCov = Cov projected into state transition space + state update noise
        //this makes the covariance wider after the motion prediction step
        pP = A*P*A.transpose() + Ex; //why do you have to do the projection rather than just multply

        //predicted var of measurment =
        //     predicted var state projected into measurement space + measurement var
        S =(C*pP*C.transpose() + Ez);

        //kalman gain: strength to apply measurement data to the state data
        //roughly total confidence the update = predicted var of the state / var of measurment
        //high confidence in measurement = high confidence in the update
        K = pP*C.transpose() * S.inverse();


        //apple the update
        //new state = predictedState + confidence in difference between measurement and predicted mesaurement
        x = px + K*(z - C*px);

        //new cov = (1-confidence of )*predictedCov
        P = (I - K*C)*pP;




        //state prediction
//        px = A*x + B*u + Ex;

        //sensor prediction
  //      pz = C*px + Ez;

        //state estimation = predicted state + scaled diff between actual and predicted measurment
    //    x = px + K*(z - pz);


//        //measurement noise - made up
//        Ex << measurementVar,0, //x is 1
//             0,measurementVar; //y is 1

//        //measurement update
//        y = z - H*x; //error between predicted and measured values
//        S = H*P*H.transpose() + Ex;//project system uncertainty into the measurement space with noise
//        K = P*H.transpose() * S.inverse();//compute scaling coefficient for the correction
//        x = x + K*y;//correct our state based off a scaled version of the error
//        P = (I - K*H)*P;//compute updated uncertainty covariance

//        //update prediction
//        x = A*x + B*u;//update state using transition matrix plus external impact
//        P = A*P*A.transpose() + Ex; //update the covariance using the transition matrix
    }


    void Test() {

//        //position measurements
//        const int measurementCount = 5;
//        Matrix<float,measurementCount,measureDim> measurement;
//        measurement <<
//           5, 0,
//           7, 1,
//           13, 4,
//           23, 9,
//           37, 16;

//        for(int i = 0; i < measurementCount; i++) {
//            Update(measurement.row(i));
//        }

//        Matrix<float, stateSize,1> ans;
//        ans <<  54.993,
//                19.9933,
//                3.99743,
//                24.998,
//                9.99829,
//                1.99943;

//        float diff = (ans-x).squaredNorm();
//        if(diff > 0.001) {
//            printf("FAILED TEST with delta %f\n", diff);
//        } else {
//            printf("PASSED TEST\n");
//        }
    }
};


#endif // KALMANFILTER_H
