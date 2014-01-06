#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "Eigen/Core"
#include "Eigen/LU"
#include "stdio.h"
#include "float.h"


using namespace Eigen;
using namespace std;

template <class T>
inline void printMatrix(const char * name,const T & m) {
    cout << name << "----\n" << m << endl;
}


inline float randf() {
    return rand()/(float)RAND_MAX;
}

inline float randfGaussian(float mu = 0, float sig =1) {
    //uses the box muller method
    float u = randf();
    float v = randf();
    return sig*sqrtf(-2*log(u))*cos(2*M_PI*v) + mu;
}

inline Vector2f randVec2fGaussian(float mu_x = 0,float mu_y = 0, float sig_x = 1, float sig_y = 1) {
    //uses the box muller method
    float u = rand()/(float)RAND_MAX;
    float v = rand()/(float)RAND_MAX;
    Vector2f result;
    result[0] = sig_x*sqrtf(-2*log(u))*cos(2*M_PI*v) + mu_x;
    result[1] = sig_y*sqrtf(-2*log(u))*sin(2*M_PI*v) + mu_y;
    return result;
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
class KalmanFilterNinjaQuail {

public:

    static const int stateSize = 2;
    static const int measureDim = 1;

    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance
    Matrix<float, stateSize,stateSize> pP; //predicted uncertainty covariance
    Matrix<float, stateSize,stateSize> A;//state1 to state2 equations
    Matrix<float, stateSize,1> B;//control input to state equations

    Matrix<float, stateSize,stateSize> Ex; //noise of a state prediction
    Matrix<float, stateSize,1> px; //state update noise

    Matrix<float, measureDim,stateSize> C; //state-to-measurement equation
    Matrix<float, measureDim,1> pz; //predicted measurement

    Matrix<float, stateSize,stateSize> I; //identity

    Matrix<float, measureDim,1> y;
    Matrix<float, measureDim,1> z;
    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;

    static const float duration = 30;
    static const float dt = 0.1f;

    KalmanFilterNinjaQuail(){

        ResetEstimation();

        //measurement noise - default to identity
        Ex =  MatrixXf::Identity(stateSize, stateSize);

        //state transition matrix
        A << 1,dt, //pos' = pos + dy*vel
             0,1; //vel' = vel

        B << dt*dt/2, dt;  //control input into state

        //measurement function
        C << 1,0; //only measure positions

        I = MatrixXf::Identity(stateSize,stateSize);

        Matrix<float, stateSize,1> Q; //quail state
        Matrix<float, stateSize,1> Qest; //quail state estimate
        Q <<  0,0;
        Qest = Q;

        //noise of measurement
        //Ez << NinjaVision_noise_mag*NinjaVision_noise_mag;

        //noise of state transition?
        Ex = MatrixXf::Identity(stateSize,stateSize);

        //state covariance
        P = Ex;
    }

    Matrix<float, stateSize,stateSize> Covariance() {
        return P;
    }

    Matrix<float, stateSize,1> State() {
        return x;
    }

    Matrix<float, stateSize,1> PredictedState() {
        return A*x;
    }

    void ResetEstimation() {
        x.setZero();

        //noise of state transition?
        Ex = MatrixXf::Identity(stateSize,stateSize);

        //state covariance
        P = MatrixXf::Identity(stateSize, stateSize)*1000;
    }



    void Update(Matrix<float, measureDim,1> z, Matrix<float, measureDim,measureDim> Ez, Matrix<float, 1,1> u) {
        //predicted state = transitionMat*state + controlMat*control
        px = A*x + B*u;

        //predictedCov = Cov projected into state transition space + state update noise
        //this makes the covariance wider after the motion prediction step
        pP = A*P*A.transpose() + Ex; //why do you have to do the projection rather than just multply

//        printMatrix("z", z.transpose());
//        printMatrix("u", u.transpose());
//        printMatrix("px", px.transpose());
//        printMatrix("pP", pP);

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
    }

};

class KalmanFilter2DPosVelAccel {

public:

    static const int stateSize = 6;
    static const int measureDim = 2;

    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance

    Matrix<float, stateSize,stateSize> Ex; //state prediction noise

    Matrix<float, stateSize,stateSize> A;//state1 to state2 equations
    Matrix<float, stateSize,1> B;//control input to state equations
    Matrix<float, measureDim,stateSize> C; //state-to-measurement equation

    Matrix<float, stateSize,stateSize> I; //identity

    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;

    KalmanFilter2DPosVelAccel(){
        ResetEstimation();

        //measurement noise - default to identity
        Ex =  MatrixXf::Identity(stateSize, stateSize);

        //measurement function
        C << 1,0,0,0,0,0,
             0,0,0,1,0,0; //only measure positions

        I = MatrixXf::Identity(stateSize,stateSize);
    }

    Matrix<float, stateSize,stateSize> Covariance() {
        return P;
    }

    Matrix<float, stateSize,1> State() {
        return x;
    }

    Vector2f Position() {
        Vector2f result;
        result[0] = x(0);
        result[1] = x(3);
        return result;
    }


    Vector2f PredictedPosition(float dt) {
        GenerateTransitionMatrix(dt);
        Matrix<float, stateSize,1> px;
        px = A*x;
        Vector2f result;
        result[0] = px(0);
        result[1] = px(3);
        return result;
    }


    Vector2f Velocity() {
        Vector2f result;
        result[0] = x(1);
        result[1] = x(4);
        return result;
    }

    void ResetEstimation() {
        x.setZero();

        //variability in prediction - i.e. how wrong is our prediction model
        Ex = MatrixXf::Identity(stateSize,stateSize);

        //initial state covariance
        P = MatrixXf::Identity(stateSize,stateSize);
    }

    void GenerateTransitionMatrix(float dt) {
        //state transition matrix
        A << 1, dt,0.5*dt*dt, 0, 0, 0, //pos' = pos + dy*vel
             0, 1, dt,        0, 0, 0, //vel' = vel
             0, 0, 1,         0, 0, 0,
             0, 0, 0,         1, dt,0.5*dt*dt,
             0, 0, 0,         0, 1, dt,
             0, 0, 0,         0, 0, 1;
    }

    void Update(Matrix<float, measureDim,1> z, Matrix<float, measureDim,measureDim> Ez, Matrix<float, 1,1> u, float dt) {

        GenerateTransitionMatrix(dt);

//        B << dt*dt/2, dt;  //control input into state

        //predicted state = transitionMat*state + controlMat*control
        x = A*x + B*u;

        //predictedCov = Cov projected into state transition space + state update noise
        //this makes the covariance wider after the motion prediction step
        P = A*P*A.transpose() + Ex; //why do you have to do the projection rather than just multply?

        //predicted var of measurment =
        //     predicted var state projected into measurement space + measurement var
        S =(C*P*C.transpose() + Ez);

        //kalman gain: strength to apply measurement data to the state data
        //roughly total confidence the update = predicted var of the state / var of measurment
        //high confidence in measurement = high confidence in the update
        K = P*C.transpose() * S.inverse();

        //apple the update
        //new state = predictedState + confidence in difference between measurement and predicted mesaurement
        x = x + K*(z - C*x);

        //new cov = (1-confidence of )*predictedCov
        P = (I - K*C)*P;
    }

};


class KalmanFilterGyro {

public:

    static const int stateSize = 9;
    static const int measureDim = 3;

    //this a 2D position with hidden velocity and acceleration on each axis
    Matrix<float, stateSize,1> x; //state estimate
    Matrix<float, stateSize,stateSize> P; //uncertainty covariance

    Matrix<float, stateSize,stateSize> Ex; //state prediction noise

    Matrix<float, stateSize,stateSize> A;//state1 to state2 equations
    Matrix<float, stateSize,1> B;//control input to state equations
    Matrix<float, measureDim,stateSize> C; //state-to-measurement equation
    Matrix<float, 2*measureDim,stateSize> Cstill; //state-to-measurement equation

    Matrix<float, stateSize,stateSize> I; //identity

    Matrix<float, measureDim,measureDim> S;
    Matrix<float, stateSize,measureDim> K;
    Matrix<float, stateSize,2*measureDim> Kstill;

    int data_counter;
    static const int data_max = 1000;
    Matrix<float,measureDim,data_max> raw_data;
    Matrix<float,measureDim,1> raw_data_sample;

    Matrix<float, measureDim,measureDim> Ez;
    Matrix<float, 2*measureDim,2*measureDim> EzStill;


    static const int deg_per_sec = 3000;


    KalmanFilterGyro(){


        ResetEstimation();

        data_counter = 0;

        //measurement noise - default to identity
        Ex =  MatrixXf::Identity(stateSize, stateSize);

        //measurement function
        C << 0,0,0,1,0,0,0,0,0,
             0,0,0,0,1,0,0,0,0,
             0,0,0,0,0,1,0,0,0; //measure velocities

        //measurement function
        Cstill <<   1,0,0,0,0,0,0,0,0,
                    0,1,0,0,0,0,0,0,0,
                    0,0,1,0,0,0,0,0,0, //measure pos and velocities
                    0,0,0,1,0,0,0,0,0,
                    0,0,0,0,1,0,0,0,0,
                    0,0,0,0,0,1,0,0,0; //measure pos and velocities


        I = MatrixXf::Identity(stateSize,stateSize);
    }

    Matrix<float, stateSize,stateSize> Covariance() {
        return P;
    }

    Matrix<float, stateSize,1> State() {
        return x;
    }

    void UpdateRawDataSample(float dt) {
        printf("Sample: %d ", data_counter);
        if(data_counter >= data_max) {
            Update(raw_data_sample,dt);
            printMatrix("x", x.transpose());
            return;
        } else {
            Matrix<float,2*measureDim,1> stillMeas;
            stillMeas.setZero();
            stillMeas.block(3,0,3,1) = raw_data_sample;
            UpdateStill(stillMeas,dt);
            printMatrix("x", x.transpose());
            printMatrix("P", P);
            data_counter++;
            return;
        }

        raw_data.col(data_counter%data_max) = raw_data_sample;
        printMatrix("gyro data", raw_data.col(data_counter%data_max).transpose());
        data_counter++;
        if(data_counter == data_max) {
            Matrix<float, measureDim,1> mean = raw_data.rowwise().sum()/data_max;
            Matrix<float, measureDim,1> variance;
            variance.setZero();
            for(int i = 0; i < data_max; i++) {
                variance += (raw_data.col(i) - mean).cwiseAbs2();
            }
            variance /= data_max-1;

            printMatrix("mean",mean.transpose());
            printMatrix("variance",variance.transpose());

            Ez = variance.asDiagonal();
            x.block(6,0,3,1) = mean;
            printMatrix("x",x.transpose());
            printMatrix("Ez",Ez);

            EzStill = MatrixXf::Identity(2*measureDim, 2*measureDim);
            EzStill.block(3,3,3,3) = Ez;
        }
    }

    Vector3f Rotation() {
        Vector3f result;
        result[0] = x(0);
        result[1] = x(1);
        result[2] = x(2);
        return result;
    }

    Vector3f RotationRate() {
        Vector3f result;
        result[0] = x(3);
        result[1] = x(4);
        result[2] = x(5);
        return result;
    }

    void ResetEstimation() {
        x.setZero();

        //variability in prediction - i.e. how wrong is our prediction model
        Ex = 5*MatrixXf::Identity(stateSize,stateSize);
        Ez = 5*MatrixXf::Identity(measureDim, measureDim);
        EzStill = 5*MatrixXf::Identity(2*measureDim, 2*measureDim);

        //initial state covariance
        P = 1000*MatrixXf::Identity(stateSize,stateSize);
    }

    void GenerateTransitionMatrix(float dt) {

        float s = dt*deg_per_sec/32768.0f;
        //state transition matrix
        A << 1, 0, 0,  s, 0, 0, -s, 0, 0, //pos' = pos + s*vel - s*bias
             0, 1, 0,  0, s, 0, 0, -s, 0,
             0, 0, 1,  0, 0, s, 0, 0, -s,
             0, 0, 0,  1, 0, 0, 0, 0, 0,
             0, 0, 0,  0, 1, 0, 0, 0, 0,
             0, 0, 0,  0, 0, 1, 0, 0, 0,
             0, 0, 0,  0, 0, 0, 1, 0, 0,
             0, 0, 0,  0, 0, 0, 0, 1, 0,
             0, 0, 0,  0, 0, 0, 0, 0, 1;

    }

    void Update(Matrix<float, measureDim,1> z, float dt) {

        GenerateTransitionMatrix(dt);
        x = A*x;
        P = A*P*A.transpose() + Ex; //why do you have to do the projection rather than just multply?
        S =(C*P*C.transpose() + Ez);
        K = P*C.transpose() * S.inverse();
        x = x + K*(z - C*x);
        P = (I - K*C)*P;
    }

    void UpdateStill(Matrix<float, 2*measureDim,1> z, float dt) {
        GenerateTransitionMatrix(dt);
        x = A*x;
        P = A*P*A.transpose() + Ex; //why do you have to do the projection rather than just multply?
        Kstill = P*Cstill.transpose() * (Cstill*P*Cstill.transpose() + EzStill).inverse();
        x = x + Kstill*(z - Cstill*x);
        P = (I - Kstill*Cstill)*P;
    }


};
#endif // KALMANFILTER_H
