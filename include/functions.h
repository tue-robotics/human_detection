#include <math.h>
using namespace std;
#include <vector>
using matrix = vector<vector<double>>;
#include <human_walking_detection/PoseVel.h>




void getLikelihood(double u, double v, double v_x, double v_y, double d_side1, double d_side2, double &likelihood);


matrix multiplyMat(matrix A, matrix B);
matrix transMat(matrix A);
matrix addMat(matrix A, matrix B, double sign);

void updateKalman(matrix A,matrix H,matrix &P,matrix Sigma_a,matrix R,matrix I,human_walking_detection::PoseVel &humanPosVel,vector<double> Z, double dt);
void initializeKalman(matrix &A,matrix &H,matrix &P,matrix &Sigma_a,matrix &R,matrix &I,double dt);

void curvedVectorField1(double x, double y, double &u, double &v, double &dist1, double &dist2);
void curvedVectorField2(double x, double y, double &u, double &v, double &dist1, double &dist2);
