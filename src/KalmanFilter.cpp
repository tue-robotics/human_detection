#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    
}

void KalmanFilter::update(hip_msgs::PoseVel &humanPosVel, vector<double> Z_in, double measurementTime) 
{
    double dt = measurementTime - tLatestUpdate_;

    /*matrix G;
    G.push_back({0.5*dt*dt});
    G.push_back({0.5*dt*dt});
    G.push_back({dt});
    G.push_back({dt});*/
    
    A_[0][2] = dt;
    A_[1][3] = dt;

    double sigma_a = 5.0; // TODO make configurable?
    matrix Q;
    Q.push_back({0.5*dt*dt*sigma_a, 0.0, 0.0, 0.0});
    Q.push_back({0.0, 0.5*dt*dt*sigma_a, 0.0, 0.0});
    Q.push_back({0.0, 0.0, dt*sigma_a, 0.0});
    Q.push_back({0.0, 0.0, 0., dt*sigma_a});

    /*matrix x_t_1;
    x_t_1.push_back({humanPosVel.x});
    x_t_1.push_back({humanPosVel.y});
    x_t_1.push_back({humanPosVel.vx});*/

    matrix S, temp, y, Z, K;
    Z.push_back({Z_in[0]}); // TODO 
    Z.push_back({Z_in[1]});

    x_ = multiplyMat(A_, x_);
    P_ = addMat(multiplyMat(multiplyMat(A_,P_), transMat(A_)), Q, 1.0);
    y = addMat(Z, multiplyMat(H_, x_), -1.0);
    S = addMat(multiplyMat(multiplyMat(H_, P_),transMat(H_)), R_, 1.0);
    K = multiplyMat(multiplyMat(P_, transMat(H_)), invMat2x2(S));
    x_ = addMat(x_, multiplyMat(K, y), 1.0);
    P_ = multiplyMat(addMat(I_, multiplyMat(K, H_), -1.0), P_);

    humanPosVel.x = x_[0][0];
    humanPosVel.y = x_[1][0];
    humanPosVel.vx = x_[2][0];
    humanPosVel.vy = x_[3][0];

    tLatestUpdate_ = measurementTime;
}

hip_msgs::PoseVel KalmanFilter::predictPos(double refTime) 
{
    double dt = refTime - tLatestUpdate_;

    // TODO incorporate with update step
    A_[0][2] = dt;
    A_[1][3] = dt;
    double sigma_a = 5.0; // TODO make configurable?

/*    matrix Q;
    Q.push_back({0.5*dt*dt*sigma_a, 0.0, 0.0, 0.0});
    Q.push_back({0.0, 0.5*dt*dt*sigma_a, 0.0, 0.0});
    Q.push_back({0.0, 0.0, dt*sigma_a, 0.0});
    Q.push_back({0.0, 0.0, 0., dt*sigma_a});*/

/*    matrix x_t_1;
    x_t_1.push_back({humanPosVel.x});
    x_t_1.push_back({humanPosVel.y});
    x_t_1.push_back({humanPosVel.vx});
    x_t_1.push_back({humanPosVel.vy});*/

    matrix x_t = multiplyMat(A_, x_);

    hip_msgs::PoseVel humanPosVel;
    humanPosVel.x = x_t[0][0];
    humanPosVel.y = x_t[1][0];
    humanPosVel.vx = x_t[2][0];
    humanPosVel.vy = x_t[3][0];

    return humanPosVel;
}

void KalmanFilter::init(double measurementTime, double initPosX, double initPosY) {
    double dt = 0.0;
    A_.push_back({1.0, 0.0, dt, 0.0});
    A_.push_back({0.0, 1.0, 0.0, dt});
    A_.push_back({0.0, 0.0, 1.0, 0.0});
    A_.push_back({0.0, 0.0, 0.0, 1.0});

    H_.push_back({1.0, 0.0, 0.0, 0.0});
    H_.push_back({0.0, 1.0, 0.0, 0.0});

    P_.push_back({1.0, 0.0, 0.0, 0.0});
    P_.push_back({0.0, 1.0, 0.0, 0.0});
    P_.push_back({0.0, 0.0, 1.0, 0.0});
    P_.push_back({0.0, 0.0, 0.0, 1.0});

    R_.push_back({0.1, 0.0});
    R_.push_back({0.0, 0.1});

    I_.push_back({1.0, 0.0, 0.0, 0.0});
    I_.push_back({0.0, 1.0, 0.0, 0.0});
    I_.push_back({0.0, 0.0, 1.0, 0.0});
    I_.push_back({0.0, 0.0, 0.0, 1.0});

    x_.push_back({initPosX});
    x_.push_back({initPosY});
    x_.push_back({0.0}); // vel x
    x_.push_back({0.0}); // vel y

    tLatestUpdate_ = measurementTime;
}

double KalmanFilter::getLatestUpdateTime()
{
    return tLatestUpdate_;
}