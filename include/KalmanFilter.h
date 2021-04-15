#ifndef HIP_KF_H_
#define HIP_KF_H_

#include <functions.h> // TODO replace with libraries
#include <hip_msgs/PoseVel.h>

class KalmanFilter
{

public:
    KalmanFilter();

    void update(hip_msgs::PoseVel &humanPosVel, vector<double> Z_in, double measurementTime);

    void init(double measurementTime, double initPosX, double initPosY);

    hip_msgs::PoseVel predictPos(double refTime);

    double getLatestUpdateTime();

    std::string toString();


protected:
    matrix I_; // Identity Matrix

    matrix A_; // System dynamics matrix

    matrix H_; // Observation matrix

    matrix P_; // State covariance matrix

    matrix R_; // Observation covariance matrix (assumed constant)

    matrix x_; // State estimate

    double tLatestUpdate_;
};

#endif //HIP_KF_H_
