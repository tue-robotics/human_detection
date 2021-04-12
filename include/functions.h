#ifndef HIP_FUNCTIONS_H_
#define HIP_FUNCTIONS_H_

#include <math.h>
using namespace std;
#include <vector>
using matrix = vector<vector<double>>;
#include <hip_msgs/PoseVel.h>

void getLikelihood(double u, double v, double v_x, double v_y, double d_side1, double d_side2, double &likelihood);

matrix multiplyMat(matrix A, matrix B);
matrix transMat(matrix A);
matrix addMat(matrix A, matrix B, double sign);
matrix invMat2x2(matrix A);

void curvedVectorField1(double x, double y, double &u, double &v, double &dist1, double &dist2);
void curvedVectorField2(double x, double y, double &u, double &v, double &dist1, double &dist2);

#endif //HIP_FUNCTIONS_H_