#include <functionsDiscretizedMap.h>

// void getLikelihood(double u, double v, double v_x, double v_y, double d_side1, double d_side2, double &likelihood) {

// }

void vectorFieldMap::pointsToLine(human_walking_detection::lines &lines) {
for (int i=0;i<2;i++) {
    lines.line[i].a = lines.line[i].y[1] - lines.line[i].y[0];
    lines.line[i].b = lines.line[i].x[0] - lines.line[i].x[1];
    lines.line[i].c = lines.line[i].y[0]*lines.line[i].x[1] - lines.line[i].x[0]*lines.line[i].y[1];
}
}

double vectorFieldMap::dist(double x1,double y1,double x2,double y2) {
    return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0));
}

double vectorFieldMap::sign(double var) {
    if (var<0.0) {
        return -1.0;
    } else if (var>0.0) {
        return 1.0;
    } else {
        return 0.0;
    }
}

void vectorFieldMap::intersectLines(human_walking_detection::lines lines, positionVars &result) {
    double a1 = lines.line[0].a;
    double b1 = lines.line[0].b;
    double c1 = lines.line[0].c;
    double a2 = lines.line[1].a;
    double b2 = lines.line[1].b;
    double c2 = lines.line[1].c;

    double D = a1*b2-a2*b1;
    if (fabs(D)<0.01) {
        result.curved=false;
        // x=[];
        // y=[];
    } else {
        result.curved=true;
        result.x=(c2*b1-c1*b2)/D;
        result.y=(c1*a2-c2*a1)/D;
    }
}

void vectorFieldMap::initializeTubeStraight(human_walking_detection::singleTube &tube) {
human_walking_detection::lines lines; 
lines.line = tube.lines;
double norm1 = sqrt(pow(lines.line[0].a,2.0) + pow(lines.line[0].b,2.0));
double norm2 = sqrt(pow(lines.line[1].a,2.0) + pow(lines.line[1].b,2.0));
double signA1 = sign(lines.line[0].a);
double signB1 = sign(lines.line[0].b);
double normVec1 = signA1*(fabs(lines.line[0].a/norm1)+fabs(lines.line[1].a/norm2))/2.0;
double normVec2 = signB1*(fabs(lines.line[0].b/norm1)+fabs(lines.line[1].b/norm2))/2.0;
human_walking_detection::transform transform;
transform.theta = atan2(normVec2,normVec1);
transform.x = lines.line[0].x[0];
transform.y = lines.line[0].y[0];

double T1_1, T1_2, T2_1, T2_2;

for (int i=0;i<2;i++) {
T1_1 = lines.line[0].x[i]-transform.x;
T1_2 = lines.line[0].y[i]-transform.y;
T2_1 = lines.line[1].x[i]-transform.x;
T2_2 = lines.line[1].y[i]-transform.y;
tube.x1.push_back(cos(-transform.theta)*T1_1 - sin(-transform.theta)*T1_2);
tube.y1.push_back(sin(-transform.theta)*T1_1 + cos(-transform.theta)*T1_2);
tube.x2.push_back(cos(-transform.theta)*T2_1 - sin(-transform.theta)*T2_2);
tube.y2.push_back(sin(-transform.theta)*T2_1 + cos(-transform.theta)*T2_2);
}

tube.transform = transform;
int max1,max2,min1,min2;
if ((tube.y2[0])>(tube.y2[1])) {
    max2 = 0;
    min2 = 1;
} else {
    max2 = 1;
    min2 = 0;
}
if ((tube.y1[0])>(tube.y1[1])) {
    max1 = 0;
    min1 = 1;
} else {
    max1 = 1;
    min1 = 0;
}


tube.dx = (tube.x2[min2]-tube.x1[min1]+tube.x2[max2]-tube.x1[max1])/2.0;
if (abs(tube.dx)>0.02) {
tube.dy.push_back(tube.y2[min2]-tube.y1[min1]);
tube.dy.push_back(tube.y2[max2]-tube.y1[max1]);

double grad[4];
grad[0] = tan(lines.line[0].dtheta[min1]);
grad[1] = tan(lines.line[1].dtheta[min2]);
grad[2] = tan(lines.line[0].dtheta[max1]);
grad[3] = tan(lines.line[1].dtheta[max2]);






double ATemp[3][3] = {{0.0, 0.0, 1.0}, {3.0*pow(tube.dx,2.0), 2.0*tube.dx, 1.0}, {pow(tube.dx,3.0), pow(tube.dx,2.0), tube.dx}};
mat::fixed<3,3> A;
for (int i=0;i<3;i++) {
    for (int j=0;j<3;j++) {
        A[j*3+i] = ATemp[i][j];
    }
}
vec b = {{grad[0]}, {grad[1]}, {tube.dy[0]}};

vec c = pinv(A) * b;
tube.C1.push_back(c[0]);
tube.C1.push_back(c[1]);
tube.C1.push_back(c[2]);

b = {{grad[2]}, {grad[3]}, {tube.dy[1]}};

c = pinv(A) * b;
tube.C2.push_back(c[0]);
tube.C2.push_back(c[1]);
tube.C2.push_back(c[2]);

double A2Temp[4][4] = {{1.0, 0.0, tube.y1[min1], 0.0},{0.0, 1.0, 0.0, tube.y1[min1]},{1.0, 0.0, tube.y1[max1], 0.0},{0.0, 1.0, 0.0, tube.y1[max1]}};
mat::fixed<4,4> A2;
for (int i=0;i<4;i++) {
    for (int j=0;j<4;j++) {
        A2[j*4+i] = A2Temp[i][j];
    }
}

vec b2 = {{1},{0},{0},{1}};

vec c2 = pinv(A2) * b2;
tube.C.push_back(c2[0]);
tube.C.push_back(c2[1]);
tube.C.push_back(c2[2]);
tube.C.push_back(c2[3]);








// tube.C1 = [0.0 0.0 1.0; 3*tube.dx^2 2*tube.dx 1; tube.dx^3 tube.dx^2 tube.dx]\[grad(1); grad(2); tube.dy(1)];
// tube.C2 = [0.0 0.0 1.0; 3*tube.dx^2 2*tube.dx 1; tube.dx^3 tube.dx^2 tube.dx]\[grad(3); grad(4); tube.dy(2)];
// tube.C = [1 0 tube.y1(min1) 0;0 1 0 tube.y1(min1);1 0 tube.y1(max1) 0;0 1 0 tube.y1(max1)]\[1;0;0;1];


tube.max = -tube.y1[min1];
tube.min = -tube.y1[max1];

tube.lines = lines.line;
} else {
    tube.empty = true;
}
}

void vectorFieldMap::initializeTubeCurved(human_walking_detection::singleTube &tube) {
human_walking_detection::lines lines; 
lines.line = tube.lines;
// determine tube basic elements
for (int i=0;i<2;i++) {
lines.line[i].R.push_back(dist(lines.line[i].x[0],lines.line[i].y[0],tube.x,tube.y));
lines.line[i].R.push_back(dist(lines.line[i].x[1],lines.line[i].y[1],tube.x,tube.y));

if (lines.line[i].R[0]>lines.line[i].R[1]) {
    lines.line[i].outter = 0;
    lines.line[i].inner = 1;
} else {
    lines.line[i].outter = 1;
    lines.line[i].inner = 0;
}

lines.line[i].theta = atan2(lines.line[i].y[lines.line[i].outter]-tube.y,lines.line[i].x[lines.line[i].outter]-tube.x);
}
tube.theta0 = lines.line[0].theta;
tube.theta_tot = lines.line[1].theta-tube.theta0;
if (fabs(tube.theta_tot)>0.02) {
    tube.empty = false;
tube.wrap = 0;
if (tube.theta_tot<-M_PI) {
    tube.theta_tot = tube.theta_tot + 2.0*M_PI; // add 2 pi for wrap other side
    lines.line[1].theta = lines.line[1].theta + 2.0*M_PI;
    tube.wrap=M_PI*2.0;
} else if (tube.theta_tot>M_PI) {
    tube.theta_tot = tube.theta_tot - 2.0*M_PI; // add 2 pi for wrap other side
    lines.line[1].theta = lines.line[1].theta - 2.0*M_PI;
    tube.wrap=-M_PI*2.0;
}
tube.max = -lines.line[0].R[lines.line[0].inner];
tube.min = -lines.line[0].R[lines.line[0].outter];
tube.lines = lines.line;
// calculate analytic solution tube function
tube.dr1 = -lines.line[0].R[lines.line[0].inner]+lines.line[1].R[lines.line[1].inner];
tube.dr2 = -lines.line[0].R[lines.line[0].outter]+lines.line[1].R[lines.line[1].outter];

double grad[4];
grad[0] = tan(-lines.line[0].dtheta[lines.line[0].inner])*lines.line[0].R[lines.line[0].inner];
grad[1] = tan(-lines.line[1].dtheta[lines.line[1].inner])*lines.line[1].R[lines.line[1].inner];
grad[2]= tan(-lines.line[0].dtheta[lines.line[0].outter])*lines.line[0].R[lines.line[0].outter];
grad[3] = tan(-lines.line[1].dtheta[lines.line[1].outter])*lines.line[1].R[lines.line[1].outter];

// double inverseMat[3][3];
// double b[3][1]=;

double ATemp[3][3] = {{0.0, 0.0, 1.0}, {3.0*pow(tube.theta_tot,2.0), 2.0*tube.theta_tot, 1.0}, {pow(tube.theta_tot,3.0), pow(tube.theta_tot,2.0), tube.theta_tot}};
mat::fixed<3,3> A;
for (int i=0;i<3;i++) {
    for (int j=0;j<3;j++) {
        A[j*3+i] = ATemp[i][j];
    }
}
vec b = {{grad[0]}, {grad[1]}, {tube.dr1}};

vec c = pinv(A) * b;
tube.C1.push_back(c[0]);
tube.C1.push_back(c[1]);
tube.C1.push_back(c[2]);

b = {{grad[2]}, {grad[3]}, {tube.dr2}};

c = pinv(A) * b;
tube.C2.push_back(c[0]);
tube.C2.push_back(c[1]);
tube.C2.push_back(c[2]);

// if (lines.line[0].R[lines.line[0].inner]<0.01) {
//     lines.line[0].R[lines.line[0].inner] = 0.0;
// }

double A2Temp[4][4] = {{1.0, 0.0, lines.line[0].R[lines.line[0].inner], 0.0},{0.0, 1.0, 0.0, lines.line[0].R[lines.line[0].inner]},{1.0, 0.0, lines.line[0].R[lines.line[0].outter], 0.0},{0.0, 1.0, 0.0, lines.line[0].R[lines.line[0].outter]}};
mat::fixed<4,4> A2;
for (int i=0;i<4;i++) {
    for (int j=0;j<4;j++) {
        A2[j*4+i] = A2Temp[i][j];
    }
}

vec b2 = {{1},{0},{0},{1}};

vec c2 = pinv(A2) * b2;
tube.C.push_back(c2[0]);
tube.C.push_back(c2[1]);
tube.C.push_back(c2[2]);
tube.C.push_back(c2[3]);

tube.signlr = -sign(tube.theta_tot);
} else {
    tube.empty = true;
}
}

void vectorFieldMap::zVal(human_walking_detection::singleTube &tube,double x,double y,bool rel,double &z, double &p, bool &inside) {
inside = true;
if (tube.curved) {
    double r = sqrt(pow(x-tube.x,2.0)+pow(y-tube.y,2.0));
    double THETA;
    if (atan2(y-tube.y,x-tube.x)<0 && tube.wrap>0) {
        THETA = atan2(y-tube.y,x-tube.x)+tube.wrap-tube.theta0;
    } else if (atan2(y-tube.y,x-tube.x)>0 && tube.wrap<0) {
        THETA = atan2(y-tube.y,x-tube.x)+tube.wrap-tube.theta0;
    } else {
        THETA = atan2(y-tube.y,x-tube.x)-tube.theta0;
    }
    tube.THETA = THETA;
    tube.r = r;
    tube.theta = atan2(y-tube.y,x-tube.x);
    double F0 = pow(THETA,3.0) * tube.C1[0] +  pow(THETA,2.0) * tube.C1[1] + THETA * tube.C1[2];
    double F1 = pow(THETA,3.0) * tube.C2[0] +  pow(THETA,2.0) * tube.C2[1] + THETA * tube.C2[2];
    z = (-r+F0*tube.C[0]+F1*tube.C[1])/(F0*tube.C[2]+F1*tube.C[3]+1.0);
    p = THETA;
    // if ~isreal(z) Checken
    //     z=0;
    // end
    if (tube.signlr>0.0 && (THETA>0.0 || THETA<tube.theta_tot)) {
        inside = false;
    } else if (tube.signlr<0.0 && (THETA<0.0 || THETA>tube.theta_tot)) {
        inside = false;
    }
    
} else {
    // xT = [cos(-tube.transform.theta) -sin(-tube.transform.theta)]*([x;y]-[tube.transform.x;tube.transform.y]);
    double xT = cos(-tube.transform.theta)*(x-tube.transform.x) - sin(-tube.transform.theta)*(y-tube.transform.y); // Checken
    // yT = [sin(-tube.transform.theta) cos(-tube.transform.theta)]*([x;y]-[tube.transform.x;tube.transform.y]);
    double yT = sin(-tube.transform.theta)*(x-tube.transform.x) + cos(-tube.transform.theta)*(y-tube.transform.y); // Checken
    // F0 = [xT^3 xT^2 xT]*tube.C1;
    // F1 = [xT^3 xT^2 xT]*tube.C2;
    double F0 = pow(xT,3.0) * tube.C1[0] +  pow(xT,2.0) * tube.C1[1] + xT * tube.C1[2];
    double F1 = pow(xT,3.0) * tube.C2[0] +  pow(xT,2.0) * tube.C2[1] + xT * tube.C2[2];
    z = (-yT+F0*tube.C[0]+F1*tube.C[1])/(F0*tube.C[2]+F1*tube.C[3]+1.0);

    if (tube.dx>0 && (xT<0 || xT>tube.dx) ) {
        inside = false;
    } else if (tube.dx<0 && (xT>0 || xT<tube.dx) ) {
        inside = false;
    }
    p = xT;
}
if (rel) {
    z = (z+tube.cTransC1)*tube.cTransC2;
}
if ((z<tube.min || z>tube.max) && !rel) {
    inside=false;
} else if ((z<0.0 || z>1.0) && rel) {
    inside=false;
}
}

human_walking_detection::line vectorFieldMap::createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2) {
    human_walking_detection::line lineTemp;
    lineTemp.x.push_back(x1);
    lineTemp.x.push_back(x2);
    lineTemp.y.push_back(y1);
    lineTemp.y.push_back(y2);
    lineTemp.dtheta.push_back(th1);
    lineTemp.dtheta.push_back(th2);
    return lineTemp;
}

void vectorFieldMap::linspace(double v1, double v2, int n, vector<double> &result) {
    n = n-1;
    double multiplier = 0.0;
    for (double i=0;i<n+1;i++) {
        result.push_back(v1 + (v2-v1)/double(n)*multiplier);
        multiplier = multiplier + 1.0;
    }
}

vectorFieldMap::positionVars vectorFieldMap::Fry(human_walking_detection::singleTube tube,double I1,double I2,bool rel) {
if (rel) {
    I2 = I2/tube.cTransC2-tube.cTransC1;
}
struct positionVars O;
if (tube.curved) {
    double THETA = I1;
    double R = I2;
    double F0 = pow(THETA,3.0)*tube.C1[0] + pow(THETA,2.0)*tube.C1[1] + THETA*tube.C1[2];
    double F1 = pow(THETA,3.0)*tube.C2[0] + pow(THETA,2.0)*tube.C2[1] + THETA*tube.C2[2];
    double r = F0*tube.C[0]+F1*tube.C[1]-R*(F0*tube.C[2]+F1*tube.C[3]+1.0);
    O.x = r*cos(tube.theta0+THETA)+tube.x;
    O.y = r*sin(tube.theta0+THETA)+tube.y;
} else {
    double xT = I1;
    double C = I2;
    double F0 = pow(xT,3.0)*tube.C1[0] + pow(xT,2.0)*tube.C1[1] + xT*tube.C1[2];
    double F1 = pow(xT,3.0)*tube.C2[0] + pow(xT,2.0)*tube.C2[1] + xT*tube.C2[2];
    double yT = F0*tube.C[0]+F1*tube.C[1]-C*(F0*tube.C[2]+F1*tube.C[3]+1.0);
    double x = cos(-tube.transform.theta)*xT + sin(-tube.transform.theta)*yT+tube.transform.x;
    double y = -sin(-tube.transform.theta)*xT + cos(-tube.transform.theta)*yT+tube.transform.y;
    O.x = x;
    O.y = y;
}
return O;
}

void vectorFieldMap::clearTube(visualization_msgs::MarkerArray map) {
    map.markers.clear();
}

void vectorFieldMap::plotTube(human_walking_detection::singleTube tube, int n1, int n2,int subID, bool rel, bool sub, visualization_msgs::MarkerArray &map) {
    vector<double> var;
    if (tube.curved) {
        linspace(0.0,tube.theta_tot,n1,var);
    } else {
        linspace(0.0,tube.dx,n1,var);
    }
    vector<double> plotVar;
    if (rel) {
        linspace(0,1,n2,plotVar);
    } else if (sub) {
        if (tube.plotZ!=101.0) {
            plotVar.push_back(tube.plotZ);
        } else {
            linspace(tube.min,tube.max,n2,plotVar);
        }
    } else {
        linspace(tube.min,tube.max,n2,plotVar);
    }    

    // plot([tube.lines(1).x(1) tube.lines(1).x(2)],[tube.lines(1).y(1) tube.lines(1).y(2)],'r');
    // plot([tube.lines(2).x(1) tube.lines(2).x(2)],[tube.lines(2).y(1) tube.lines(2).y(2)],'r');
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/semanticMap";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time();
    marker.ns = "tubeSegments";
    // marker.type = visualization_msgs::Marker::POINTS;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.scale.x = 0.1;
    // marker.scale.y = 0.2;
    // marker.scale.z = 0.1;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::Point sample;


    vectorFieldMap::positionVars O;
    for (int i=0;i<plotVar.size();i++) {
        for (int j = 0;j<var.size();j++) {
            O = Fry(tube,var[j],plotVar[i],rel);
            sample.x  = O.x;
            sample.y = O.y;
            sample.z = 0.0;
            marker.points.push_back(sample);
        }
    marker.id = subID + i;
    map.markers.push_back(marker);
    marker.points.clear();
    }
} 

void vectorFieldMap::linesToTube(human_walking_detection::lines &lines,human_walking_detection::tubes &tubes,double id,TBCS TBC,bool marked) {
    struct positionVars result;
    human_walking_detection::lines linesTemp;
    human_walking_detection::singleTube tubeTemp;
    human_walking_detection::tube tube;
for (int element = 0;element<lines.line.size()-1;element++) {
    linesTemp.line.push_back(lines.line[element]);
    linesTemp.line.push_back(lines.line[element+1]);
    pointsToLine(linesTemp);
    intersectLines(linesTemp,result);
    tubeTemp.curved = result.curved;
    tubeTemp.x = result.x;
    tubeTemp.y = result.y;
    tubeTemp.lines = linesTemp.line;
    tubeTemp.id = id;
    tubeTemp.dir = 1;
    if (tubeTemp.curved) {
        initializeTubeCurved(tubeTemp);
    } else {
        initializeTubeStraight(tubeTemp);
    }

    if (!tubeTemp.empty) {
       if (tubeTemp.curved) {
            tubeTemp.pTot = tubeTemp.theta_tot;
       } else {
            tubeTemp.pTot = tubeTemp.dx;
        } 

        if (TBC.i==-1) {
            // tubes.tube[length(tube)+1] = tubeTemp;
            tubeTemp.marked = marked;
            tubeTemp.plotZ = 101.0;
            tube.main = tubeTemp;
            tubes.tube.push_back(tube); // Checken
            // tubes.tube[length(tube)].subTubes{1} = {};
            // tubes.tube[length(tube)].subTubes{2} = {};
            // tubes.tube[length(tube)].subTubes{3} = {};
            // tubes.tube[length(tube)].subTubesB{1} = {};
            // tubes.tube[length(tube)].subTubesB{2} = {};
            // tubes.tube[length(tube)].subTubesB{3} = {};
            // tubes.tube[length(tube)].marked = marked;
        } else {
            // index = findPointInTube(tubes,(linesTemp(2).x(2)+linesTemp(1).x(1))/2,(linesTemp(2).y(2)+linesTemp(1).y(1))/2); TODO
            int index[1] = {0};
            if (TBC.oneSide) {
                double z,p;
                bool inside;
                zVal(tubeTemp,linesTemp.line[0].x[0],linesTemp.line[0].y[0],0,z,p,inside);
                tubeTemp.plotZ = z;
            } else {
                tubeTemp.plotZ = 101.0;
            }
            if (TBC.border) {
                if (TBC.i==0) {
                    tubes.tube[index[0]].subTubesB0.push_back(tubeTemp);
                } else if (TBC.i==1) {
                    tubes.tube[index[0]].subTubesB1.push_back(tubeTemp);
                } else {
                    tubes.tube[index[0]].subTubesB2.push_back(tubeTemp);
                }
            } else {
                if (TBC.i==0) {
                    tubes.tube[index[0]].subTubes0.push_back(tubeTemp);
                } else if (TBC.i==1) {
                    tubes.tube[index[0]].subTubes1.push_back(tubeTemp);
                } else {
                    tubes.tube[index[0]].subTubes2.push_back(tubeTemp);
                }
                // tube{index(1)}.subTubes{TBC.i+1}{length(tube{index(1)}.subTubes{TBC.i+1})+1} = tubeTemp;
                // tubes.tube[index[0]].subTubes[TBC.i].subTubes.push_back(tubeTemp);
                // tubes.tube[index[0]].subTubes[TBC.i].push_back(tubeTemp);
            }
        }
    }
    tubeTemp = {};
}
lines.line = {};
}

void vectorFieldMap::readMap(visualization_msgs::MarkerArray &markers) {
    markers = staticMap;
}

void vectorFieldMap::initializeMap() {
    cout<<"initialize map..."<< endl;
    human_walking_detection::human human;
    human_walking_detection::robot robot;
    TBCS TBC;
    human_walking_detection::lines lines;
    human_walking_detection::line lineTemp;

    robot.x = 4.5;
    robot.y = 8.4;
    robot.l = 1.5;
    robot.b = 0.75;
    robot.theta = 0.0;
    robot.df = 1.0;
    robot.db = 1.5;
    robot.dsMax = 1.0;
    robot.dMax = 2.5;

    human.x = 6.5;
    human.y = 8.2;
    human.r = 3.0;
    human.dMax = 4.0;

    TBC.i = -1;
    
    // lineTemp = createLineTemp(0.0,0.0,0.0,2.5);
    // lineTemp.dtheta.push_back(0.0);
    // lineTemp.dtheta.push_back(0.0);
    // lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(0.0,2.2,0.0,0.0);
    // lineTemp.dtheta.push_back(0.0);
    // lineTemp.dtheta.push_back(0.0);
    // lines.line.push_back(lineTemp);

    // linesToTube(lines,globalTube,0,TBC,1);
    // lines.line.push_back(lineTemp);
    
    // lineTemp = createLineTemp(0.0,2.5,-0.5,-0.5);
    // lineTemp.dtheta.push_back(0.0);
    // lineTemp.dtheta.push_back(0.0);
    // lines.line.push_back(lineTemp);
    // linesToTube(lines,globalTube,0,TBC,1);





    // plotTube(globalTube.tube[0].main,20,2,0,0,0,staticMap);
    // plotTube(globalTube.tube[1].main,20,2,10,0,0,staticMap);
    lineTemp = createLineTemp(-5.9,-5.9,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(-8.4,-8.4,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(-8.5,-8.5,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,1);

    lineTemp = createLineTemp(-5.9,-5.9,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(-5.9,-4.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(-5.9,-4.5,2.6,2.6,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(-5.9,-5.9,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(-4.5,-4.5,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(-5.9,-4.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(-4.5,-4.5,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,0.0,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.2,0.0,0.0,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.2,2.2,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(0.0,2.2,0.0,0.0,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.2,-0.1,-0.1,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(0.0,0.0,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(2.2,2.2,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lines.line.push_back(lineTemp);
    lineTemp = createLineTemp(2.5,2.5,0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(0.0,0.0,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.5,7.3,7.3,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.5,9.3,9.3,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.5,9.4,9.4,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.3,0.0,0.0,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(0.0,2.5,7.3,7.3,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(2.5,2.5,7.3,9.3,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.5,4.8,7.3,7.3,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.5,4.8,7.2,7.2,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(2.5,4.8,7.3,7.3,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(4.8,4.8,7.3,9.3,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.5,2.5,7.3,9.3,0.0,0.0);
        lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.5,9.3,9.3,0.0,0.0);
        lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);

    
    lineTemp = createLineTemp(4.8,4.8,7.3,9.3,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(7.3,7.3,7.3,9.3,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(7.3,9.5,7.3,7.3,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(7.3,9.5,2.5,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(9.5,9.5,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(9.6,9.6,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(9.5,9.5,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(7.3,7.3,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(7.3,9.3,2.5,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(7.3,7.3,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(6.88,6.88,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(4.8,4.8,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(5.43,6.25,0.0,0.0,M_PI/3.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(5.43,6.25,-0.1,-0.1,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(5.43,6.25,0.0,0.0,0.0,-M_PI/3.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(6.88,6.88,0.0,2.5,M_PI/3.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(2.5,4.8,2.5,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
        lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.5,4.8,2.6,2.6,0.0,0.0);
        lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(2.5,4.8,2.5,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(4.8,4.8,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);
    
    lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(4.8,4.8,0.0,2.5,0.0,0.0);
        lines.line.push_back(lineTemp);
    linesToTube(lines,globalTube,0,TBC,0);


    for (int i=0;i<globalTube.tube.size();i++) {
        plotTube(globalTube.tube[i].main,20,2,5*i,0,0,staticMap);
    }


}