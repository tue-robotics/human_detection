#include "functions.h"

void getLikelihood(double u, double v, double v_x, double v_y, double d_side1, double d_side2, double &likelihood) {
    double t = 1.0; // time to constraint remaining inside tube
    double sideMargin = 0.5; // higher is more conservative on lateral speed constraints (has to follow tube more)
    double v_min = 0.1; // minimal absolute speed to remain "moving" towards goal
    double speedMargin = 0.5; // higher is more conservative on minimal speed constraint

    double v_T =  v_x*u + v_y*v;
    double v_L =  -v_x*v + v_y*u;
    // cout<<">>>>>>>>>speed tube"<< v_T<<"speed lateral"<<v_L<<endl;
    likelihood = v_T/(v_T+fabs(v_L));
    // Minimal progression constraint
    if (v_T < v_min+speedMargin) {
        if (v_T < v_min) {
            likelihood = 0;
        } else {
            likelihood = likelihood * (1/speedMargin*(v_T-v_min));
        }
    }
    // Remain inside tube constraint 1/2 (d_side1 supposedly negative)
    if (v_L < d_side1/t+sideMargin) {
        if (v_L < d_side1/t) {
            likelihood = 0;
        } else {
            likelihood = likelihood * (+1/sideMargin*(v_L-d_side1/t));
        }
    }
    // Remain inside tube constraint 2/2 (d_side2 supposedly possitive)
    if (v_L > d_side2/t-sideMargin) {
        if (v_L > d_side2/t) {
            likelihood = 0;
        } else {
            likelihood = likelihood * (-1/sideMargin*(v_L-d_side2/t));
        }
    }
}


// void multiplyMat(double A,double b[],double &x[]) {

// }

matrix multiplyMat(matrix A, matrix B) {
    vector<double> temp;
    matrix C;
    for(int i = 0; i < A.size(); ++i)
        {
            for(int j = 0; j < B[0].size(); ++j)
            {
                temp.push_back(0.0);
                for(int k=0; k<A[0].size(); ++k)
                {
                    temp[j] += A[i][k] * B[k][j];
                }
            }
            C.push_back(temp);
            temp.clear();
        }
    return C;
}

matrix addMat(matrix A, matrix B, double sign) {
    vector<double> temp;
    matrix C;
    for(int i = 0; i < A.size(); ++i)
        {
            for(int j = 0; j < A[0].size(); ++j)
            {
                temp.push_back(A[i][j]+sign*B[i][j]);
            }
            C.push_back(temp);
            temp.clear();
        }
    return C;
}

matrix transMat(matrix A) {
    vector<double> temp;
    matrix C;
    for(int j = 0; j < A[0].size(); ++j)
        {
            for(int i = 0; i < A.size(); ++i)
            {
                temp.push_back(A[i][j]);
            }
            C.push_back(temp);
            temp.clear();
        }
    return C;
}

void printMat(matrix A) {
    for (int i=0; i<A.size();i++) {
        for (int j=0; j<A[0].size();j++) {
            cout<<A[i][j]<<" ";
        }
        cout<<endl;
    }
}

matrix invMat2x2(matrix A) {
    matrix B;
    double det;
    det = A[0][0]*A[1][1]-A[0][1]*A[1][0];
    B.push_back({A[1][1]/det,-A[1][0]/det});
    B.push_back({-A[0][1]/det,A[0][0]/det});
    return B;
}