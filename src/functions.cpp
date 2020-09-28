#include <functions.h>

void getLikelihood(double u, double v, double v_x, double v_y, double d_side1, double d_side2, double &likelihood) {
    double t = 1.0; // time to constraint remaining inside tube
    double sideMargin = 0.2; // higher is more conservative on lateral speed constraints (has to follow tube more)
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


void initializeKalman(matrix &A,matrix &H,matrix &P,matrix &Sigma_a,matrix &R,matrix &I,double dt) {
    A.push_back({1.0, 0.0, dt, 0.0});
    A.push_back({0.0, 1.0, 0.0, dt});
    A.push_back({0.0, 0.0, 1.0, 0.0});
    A.push_back({0.0, 0.0, 0.0, 1.0});

    H.push_back({1.0, 0.0, 0.0, 0.0});
    H.push_back({0.0, 1.0, 0.0, 0.0});

    P.push_back({1.0, 0.0, 0.0, 0.0});
    P.push_back({0.0, 1.0, 0.0, 0.0});
    P.push_back({0.0, 0.0, 1.0, 0.0});
    P.push_back({0.0, 0.0, 0.0, 1.0});



    // double sigma_a=5.0;

    // Sigma_a.push_back({sigma_a*sigma_a, 0, 0, 0});
    // Sigma_a.push_back({0, sigma_a*sigma_a, 0, 0});
    // Sigma_a.push_back({0, 0, sigma_a*sigma_a, 0});
    // Sigma_a.push_back({0, 0, 0, sigma_a*sigma_a});
    


    // Q.push_back({0.4, 0.0, 0.0, 0.0});
    // Q.push_back({0.0, 0.4, 0.0, 0.0});
    // Q.push_back({0.0, 0.0, 0.4, 0.0});
    // Q.push_back({0.0, 0.0, 0.0, 0.4});
    // cout<<"Q"<<endl;
    // printMat(Q);

    R.push_back({0.1, 0.0});
    R.push_back({0.0, 0.1});

    I.push_back({1.0, 0.0, 0.0, 0.0});
    I.push_back({0.0, 1.0, 0.0, 0.0});
    I.push_back({0.0, 0.0, 1.0, 0.0});
    I.push_back({0.0, 0.0, 0.0, 1.0});
}


void updateKalman(matrix A,matrix H,matrix &P,matrix Sigma_a,matrix R,matrix I,human_walking_detection::PoseVel &humanPosVel,vector<double> Z_, double dt) {

        // double x_t[4] = {}; //input
        matrix G;
        G.push_back({0.5*dt*dt});
        G.push_back({0.5*dt*dt});
        G.push_back({dt});
        G.push_back({dt});
        
        A[0][2]=dt;
        A[1][3]=dt;
        matrix Q;
        double sigma_a=5.0;

    //     Q = multiplyMat(multiplyMat(G,transMat(G)),Sigma_a);
        Q.push_back({0.5*dt*dt*sigma_a, 0.0, 0.0, 0.0});
        Q.push_back({0.0, 0.5*dt*dt*sigma_a, 0.0, 0.0});
        Q.push_back({0.0, 0.0, dt*sigma_a, 0.0});
        Q.push_back({0.0, 0.0, 0., dt*sigma_a});
        // cout<<"Values P: "<<endl;
        // printMat(P);
        matrix x_t_1;
        
        x_t_1.push_back({humanPosVel.x});
        x_t_1.push_back({humanPosVel.y});
        x_t_1.push_back({humanPosVel.vx});
        x_t_1.push_back({humanPosVel.vy});

        // cout<<"values x t-1: "<<endl;
        // printMat(x_t_1);

        matrix x_t, S, temp, y, Z, K;

        Z.push_back({Z_[0]});
        Z.push_back({Z_[1]});

        // cout<<"Z values: "<<Z[0][0]<<", "<<Z[1][0]<<endl;



        x_t = multiplyMat(A,x_t_1);

        P = addMat(multiplyMat(multiplyMat(A,P),transMat(A)),Q,1.0);
        // cout<<"pre values P: "<<endl;
        // printMat(P);

        y = addMat(Z,multiplyMat(H,x_t),-1.0);

        // cout<<"values y: "<<endl;
        // printMat(y);


        S = addMat(multiplyMat(multiplyMat(H,P),transMat(H)),R,1.0);

        // cout<<"values S: "<<endl;
        // printMat(S);

        // cout<<"inv S: "<<endl;
        // printMat(invMat2x2(S));

        K = multiplyMat(multiplyMat(P,transMat(H)),invMat2x2(S));

        // cout<<"values K: "<<endl;
        // printMat(K);


        // cout<<"values Ky: "<<endl;
        // printMat(multiplyMat(K,y));


        x_t = addMat(x_t, multiplyMat(K,y),1.0);

        P = multiplyMat(addMat(I,multiplyMat(K,H),-1.0),P);

        // cout<<"values x: "<<endl;
        // printMat(x_t);


        // cout<<"values Z: "<<endl;
        // printMat(Z);


        // cout<<"values P: "<<endl;
        // printMat(P);



        humanPosVel.x = x_t[0][0];
        humanPosVel.y = x_t[1][0];
        humanPosVel.vx = x_t[2][0];
        humanPosVel.vy = x_t[3][0];


}

