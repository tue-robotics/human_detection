#include <functionsDiscretizedMap.h>
#include <functions.h>


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

    if (tube.dx>0.0 && (xT<0.0 || xT>tube.dx) ) {
        inside = false;
    } else if (tube.dx<0.0 && (xT>0.0 || xT<tube.dx) ) {
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
    result.clear();
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

void vectorFieldMap::plotTube(human_walking_detection::singleTube tube, int n1, int n2,int subID, bool rel, bool sub,double r, double g, double b, double a, string ns,visualization_msgs::MarkerArray &map) {
    vector<double> var;
    if (tube.curved) {
        linspace(0.0,tube.theta_tot,n1,var);
    } else {
        linspace(0.0,tube.dx,n1,var);
    }
    vector<double> plotVar;
    if (rel) {
        linspace(0.0,1.0,n2,plotVar);
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
    marker.ns = ns;
    // marker.type = visualization_msgs::Marker::POINTS;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
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

void vectorFieldMap::readMap(visualization_msgs::MarkerArray &staticMarkers,visualization_msgs::MarkerArray &dynamicMarkers) {
    staticMarkers = staticMap;
    dynamicMarkers = dynamicMap;
}

void vectorFieldMap::FuvCurved(double dphidr,double dphidtheta,double theta,double r,double signlr,double &u, double &v) {
    u = signlr* (cos(theta)*-dphidtheta - r*sin(theta)*dphidr);
    v = signlr* (sin(theta)*-dphidtheta + r*cos(theta)*dphidr);
    double l = sqrt(pow(u,2.0)+pow(v,2.0));
    // norm([u,v]);
    u=u/l;
    v=v/l;
}

void vectorFieldMap::FuvStraight(double dphidx,double dphidy,human_walking_detection::singleTube tube,double &u, double &v) {
    double uT = -dphidy;
    double vT = dphidx;
    u = cos(tube.transform.theta)*uT - sin(tube.transform.theta)*vT;
    v = sin(tube.transform.theta)*uT + cos(tube.transform.theta)*vT;
    double l = sqrt(pow(u,2.0)+pow(v,2.0));
    u=u/l;
    v=v/l;
}

void vectorFieldMap::derivativeF(human_walking_detection::singleTube tube, double x, double y, double &dphidx, double &dphidy) {
double c_1,c_2,c_3,c_4,c_11,c_12,c_13,c_21,c_22,c_23;
c_1 = tube.C[0];
c_2 = tube.C[1];
c_3 = tube.C[2];
c_4 = tube.C[3];
c_11 = tube.C1[0];
c_12 = tube.C1[1];
c_13 = tube.C1[2];
c_21 = tube.C2[0];
c_22 = tube.C2[1];
c_23 = tube.C2[2];
dphidx = ((3.0 *c_21 *pow(x,2.0) + 2.0 *c_22 *x + c_23) *(c_4 *y + c_2) + c_3 *(c_12 *x *(c_2 *(c_21 *pow(x,3.0) - c_23 *x) + 2.0 *y) - c_11 *pow(x,2.0) *(c_2 *x *(c_22 *x + 2.0 *c_23) - 3.0 *y) + c_13 *(c_2 *pow(x,2.0) *(2.0 *c_21 *x + c_22) + y)) + c_1 *(c_12 *x *(c_4 *(c_23 *x - c_21 *pow(x,3.0)) + 2.0) + c_11 *pow(x,2.0) *(c_4 *x *(c_22 *x + 2.0 *c_23) + 3.0) - c_13 *(c_4 *pow(x,2.0) *(2.0 *c_21 *x + c_22) - 1.0)))/pow(c_3 *x *(c_11 *pow(x,2.0) + c_12 *x + c_13) + c_4 *x *(c_21 *pow(x,2.0) + c_22 *x + c_23) + 1.0,2.0); // checken
dphidy = -1.0/(c_3 *x *(c_11 *pow(x,2.0) + c_12 *x + c_13) + c_4 *x *(c_21 *pow(x,2.0) + c_22 *x + c_23) + 1.0);
}

void vectorFieldMap::calcGradient(human_walking_detection::singleTube tube,double x, double y, double &u, double &v) {
    double z,p,dphidtheta,dphidr,dphidx,dphidy;
    bool inside;
    zVal(tube,x,y,false,z,p,inside);
    if (tube.curved) {
        derivativeF(tube,tube.THETA,tube.r,dphidtheta,dphidr);
        FuvCurved(dphidr,dphidtheta,tube.theta,tube.r,tube.signlr,u,v);
    } else {
        // xT = [cos(-tube.transform.theta) -sin(-tube.transform.theta)]*([x;y]-[tube.transform.x;tube.transform.y]);
        double xT = cos(-tube.transform.theta)*(x-tube.transform.x) - sin(-tube.transform.theta)*(y-tube.transform.y); // Checken

        // yT = [sin(-tube.transform.theta) cos(-tube.transform.theta)]*([x;y]-[tube.transform.x;tube.transform.y]);
        double yT = sin(-tube.transform.theta)*(x-tube.transform.x) + cos(-tube.transform.theta)*(y-tube.transform.y); // Checken

        derivativeF(tube,xT,yT,dphidx,dphidy);
        FuvStraight(dphidx,dphidy,tube,u,v);
        if (tube.dx<0.0) {
            u = -u;
            v = -v;
        }
    }
    if (tube.dir == -1.0) {
        u = -u;
        v = -v;
    }
}

void vectorFieldMap::findPointInTube(human_walking_detection::tubes tubes, double x, double y, vector<int> &index) {
double z,p;
bool inside;
index.clear();
for (int ind = 0;ind<tubes.tube.size();ind++) {
    zVal(tubes.tube[ind].main,x,y,false,z,p,inside);
    if (inside) {
        index.push_back(ind);
    }
}
}

void vectorFieldMap::createGraph(human_walking_detection::tubes tube, graph &G) {
    double step = 1.0/1000.0;
    vector<vector<double>> A;
    vector<double> row;
    vector<bool> AMarked;
    for (int i=0;i<tube.tube.size();i++) {
        row.push_back(0.0);
        AMarked.push_back(false);
    }

    for (int i=0;i<tube.tube.size();i++) {
        A.push_back(row);
    }
    vectorFieldMap::positionVars O;
    double u,v,z,s,p;
    vector<int> index;
    vector<double> pVars,sVars;
    pVars.push_back(0.0);
    pVars.push_back(0.0);
    sVars.push_back(step);
    sVars.push_back(-step);
    bool valid,inside;
    for (int i = 0;i<tube.tube.size();i++) {
        pVars[1] = tube.tube[i].main.pTot;
        // for (double p = 0.0;p<1.1*tube.tube[i].main.pTot;p = p + tube.tube[i].main.pTot) {
        for (int pInd = 0;pInd<2;pInd++) {
            p = pVars[pInd];
            // cout<<"p test: "<< p << endl;
            O = Fry(tube.tube[i].main,p,(tube.tube[i].main.min+tube.tube[i].main.max)/2.0,false);
            calcGradient(tube.tube[i].main,O.x,O.y,u,v);
            // for (double s=step;s>0.0;s=-step) {
            for (int sInd = 0;sInd<2;sInd++) {
                s = sVars[sInd];
                findPointInTube(tube,O.x+u*s,O.y+v*s,index);
                valid = true;
                for (int j = 0;j<index.size();j++) {
                    zVal(tube.tube[index[j]].main,O.x+u*s,O.y+v*s,0,z,p,inside);
                    if ((fabs(p-tube.tube[index[j]].main.pTot)>0.05) && (fabs(p)>0.05)) {
                        index[j] = 0;
                    }
                    if (index[j]==i) {
                        valid = false;
                    }
                }
                if (valid) {
                    for (int j = 0;j<index.size();j++) {
                        if (index[j]!=0) {
                            if (s>0.0) {
                                A[index[j]][i] = 1.0;
                            } else {
                                A[index[j]][i] = -1.0;                        
                            }
                        }
                    }
                }
            }
        }   
    }

    for (int i=0; i<A.size();i++) {
        A[i][i] = 0.0;
        if (tube.tube[i].main.marked) {
            AMarked[i] = true;
        }
    }
    G.A = A;
    G.marked = AMarked;
}

void vectorFieldMap::createGraphShort(human_walking_detection::tubes tube, graph &G) {
vector<bool> visited;
struct positionVars O;
for (int i=0;i<G.A.size();i++) {
    visited.push_back(false);
}
vector<int> coupledTemp,ALink;
vector<vector<int>> coupled;
vector<int> index;
// coupledTemp = [];
// coupled = [];
int shorten = 0;
for (int i = 0;i<G.A.size();i++) {
    if (!visited[i]) {
        O = Fry(tube.tube[i].main,tube.tube[i].main.pTot/2.0,(tube.tube[i].main.max+tube.tube[i].main.min)/2.0,false);
        findPointInTube(tube,O.x,O.y,index);
        for (int j = 0; j<index.size(); j++) {
            if ((i!=index[j]) && !visited[index[j]]) {
                shorten = shorten + 1;
                visited[index[j]]=true;
                coupledTemp.push_back(index[j]);
            }
        }
        if (coupledTemp.size()>0) {
            coupledTemp.push_back(i);
            coupled.push_back(coupledTemp);
            visited[i]=true;
        }
        coupledTemp.clear();
    }
}

// ALink = zeros(length(visited),1);
for (int i=0;i<visited.size();i++) {
    ALink.push_back(0);
}


int indMat = 0;
for (int i = 0; i<visited.size(); i++) {
    if (visited[i]) {
        for (int j = 0; j< coupled.size();j++) {
            coupledTemp = coupled[j];
            if (i==coupledTemp.back()) {
                ALink[i] = indMat;
            }
            for (int k = 0; k<coupledTemp.size()-1;k++) {
                if (i==coupledTemp[k]) {
                    ALink[i] = ALink[coupledTemp.back()];
                    indMat = indMat - 1;
                }
            }
        }
    }
    if (ALink[i]==0) {
        ALink[i] = indMat;
    }
    indMat = indMat + 1;
}

int lenA = G.A.size();
// AShort = zeros(lenA-shorten,lenA-shorten);
vector<vector<double>> AShort;
vector<double> row;
for (int i=0;i<lenA-shorten;i++) {
    row.push_back(0.0);
}
for (int i=0;i<lenA-shorten;i++) {
    AShort.push_back(row);
}

for (int i = 0;i<lenA;i++) {
    for (int j = 0; j<lenA; j++) {
        double val = fabs(AShort[ALink[i]][ALink[j]]) + fabs(G.A[i][j]);
        if (val>0.0) {
            AShort[ALink[i]][ALink[j]] = 1.0;
        } else {
            AShort[ALink[i]][ALink[j]] = 0.0;            
        }
    }
}

for (int i = 0;i<AShort.size();i++) {
    G.AShort.push_back(AShort[i]);
}

for (int i = 0;i<ALink.size();i++) {
    G.ALink.push_back(ALink[i]);
}

// G.AShort = AShort;

// G.ALink = ALink;
}

bool vectorFieldMap::considerHuman(human_walking_detection::human person,human_walking_detection::robot robot) {
    double dRP = dist(person.x,person.y,robot.x,robot.y);
    bool consider = true;
    if (dRP>5.0) {
        consider = false;
    }
    return consider;
}

void vectorFieldMap::nextTube(int index,graph G,vector<int> &toTube, vector<double> &direction) {
    toTube.clear();
    direction.clear();
    for (int i = 0; i<G.A.size();i++) {
        if (G.A[i][index]!=0) {
            toTube.push_back(i);
            direction.push_back(G.A[i][index]);
        }
    }
}

bool vectorFieldMap::inAvailableTube(vector<bool> openTubes,graph G,int toTube) {
    bool considered = false;
    if (openTubes[G.ALink[toTube]]) {
        considered = true;
    } 
    return considered;
}

void vectorFieldMap::distEnd(double x,double y,vector<recursiveWalkStore> &store,vector<recursiveWalk> prev,vector<recursiveWalk> &next,human_walking_detection::tubes tube,graph G,vector<bool> openTubes,double dMax) {
    // next = [];
    vector<int> indexTemp;
    vector<int> index;
    next.clear();


    vector<int> to;
    vector<double> d_old;
    vector<double> pStartStore;
    vector<double> zStartStore;
    vector<int> storeElementTemp;
    vector<vector<int>> storeElement;

    for (int i = 0; i<prev.size();i++) {
        to.push_back(prev[i].toTube);
        d_old.push_back(prev[i].distance);
        pStartStore.push_back(prev[i].pEndStore);
        zStartStore.push_back(prev[i].zEndStore);
        storeElementTemp = prev[i].storeElement;
        storeElement.push_back(storeElementTemp);
    }

    if (to.size()>0) {
        index = to;
    } else if (x!=-1.0 && y!=-1.0) {
        findPointInTube(tube,x,y,indexTemp);
        for (int i = 0; i<indexTemp.size();i++) {
            if (openTubes.size()>0) {
                if (inAvailableTube(openTubes,G,indexTemp[i])) {
                    storeElementTemp.push_back(indexTemp[i]);
                    storeElement.push_back(storeElementTemp);
                    storeElementTemp.clear();
                    d_old.push_back(0.0);
                    index.push_back(indexTemp[i]);
                }
            } else{
                storeElementTemp.push_back(indexTemp[i]);
                storeElement.push_back(storeElementTemp);
                storeElementTemp.clear();
                d_old.push_back(0.0);
                index.push_back(indexTemp[i]);
            }
        }
    }
    storeElementTemp.clear();
    vector<int> toTubeTemp;
    vector<double> dirTemp,pVar;
    bool inside;
    double z, p, pTot,d;
    struct positionVars O,O_new;
    struct recursiveWalkStore storeTemp;
    struct recursiveWalk nextTemp;
    bool checked;
    // vector<struct recursiveWalkStore> store;
    for (int i = 0; i<index.size();i++) {
        nextTube(index[i],G,toTubeTemp,dirTemp);
        for (int j = 0; j<toTubeTemp.size();j++) {
            if (to.size()>0) {
                z = zStartStore[i];
                p = pStartStore[i];
            } else {
                zVal(tube.tube[index[i]].main,x,y,0,z,p,inside);
            }
            if (dirTemp[j]==1.0) {
                pTot = tube.tube[index[i]].main.pTot;
            } else {
                pTot = 0.0;
            }
            
            linspace(p,pTot,10,pVar);



            O = Fry(tube.tube[index[i]].main,pVar[0],z,0);
            d = 0.0;
            for (int k = 1; k<pVar.size();k++) {
                O_new = Fry(tube.tube[index[i]].main,pVar[k],z,0);
                d = d + dist(O.x,O.y,O_new.x,O_new.y);
                O = O_new;
            }

            if (tube.tube[toTubeTemp[j]].main.marked) {
                storeTemp.index = toTubeTemp[j];
                storeTemp.dTot = d+d_old[i];
                zVal(tube.tube[toTubeTemp[j]].main,O.x,O.y,0,z,p,inside);
                storeTemp.pStore = p;
                storeElementTemp = storeElement[i];
                storeElementTemp.push_back(toTubeTemp[j]);
                storeTemp.path = storeElementTemp;
                storeElementTemp.clear();
                store.push_back(storeTemp);
            } else if ((d>0.01) && ((d + d_old[i])<dMax)) {
                checked = false;
                if (openTubes.size()==0) {
                    checked = true;
                } else if (inAvailableTube(openTubes,G,toTubeTemp[j])) {
                    checked = true;
                } else {
                    storeTemp.index = toTubeTemp[j];
                    storeTemp.dTot = d+d_old[i];
                    zVal(tube.tube[toTubeTemp[j]].main,O.x,O.y,0,z,p,inside);
                    storeTemp.pStore = p;
                    storeElementTemp = storeElement[i];
                    storeElementTemp.push_back(toTubeTemp[j]);
                    storeTemp.path = storeElementTemp;
                    storeElementTemp.clear();
                    // storeTemp.path = [storeElement{i} toTubeTemp(j)];
                    store.push_back(storeTemp);
                }
                if (checked) {
                    nextTemp.distance = d+d_old[i];
                    nextTemp.dir = dirTemp[j];
                    nextTemp.toTube = toTubeTemp[j];
                    nextTemp.fromTube = index[i];
                    zVal(tube.tube[toTubeTemp[j]].main,O.x,O.y,0,z,p,inside);
                    nextTemp.pEndStore = p;
                    nextTemp.zEndStore = z;
                    storeElementTemp = storeElement[i];
                    storeElementTemp.push_back(toTubeTemp[j]);
                    nextTemp.storeElement = storeElementTemp;
                    storeElementTemp.clear();
                    next.push_back(nextTemp);
                }
            } else if (d>0.01) {
                storeTemp.index = toTubeTemp[j];
                storeTemp.dTot = d+d_old[i];
                zVal(tube.tube[toTubeTemp[j]].main,O.x,O.y,0,z,p,inside);
                storeTemp.pStore = p;
                storeElementTemp = storeElement[i];
                storeElementTemp.push_back(toTubeTemp[j]);
                storeTemp.path = storeElementTemp;
                storeElementTemp.clear();
                // storeTemp.path = [storeElement{i} toTubeTemp(j)];
                store.push_back(storeTemp);
            }
        }
    }
}

void vectorFieldMap::recursiveWalkA(vector<recursiveWalkStore> &store,vector<recursiveWalk> next,human_walking_detection::tubes tube,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax) {
    if (seenTubes.size()>0) {
        for (int i = 0; i<next.size();i++) {
            seenTubes[G.ALink[next[i].toTube]] = true;
        }
    }


    distEnd(-1.0,-1.0,store,next,next,tube,G,openTubes,dMax);
    if (next.size()>0) {
        recursiveWalkA(store,next,tube,G,seenTubes,openTubes,dMax);
    }
}

void vectorFieldMap::createDynamicAsOI(human_walking_detection::tubes tube,double xPR, double yPR,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax,vector<recursiveWalkStore> &store) {
vector<struct recursiveWalk> next,prev;
distEnd(xPR,yPR,store,prev,next,tube,G,openTubes,dMax);
if (seenTubes.size()>0) {
    for (int i = 0; i<next.size();i++) {
        seenTubes[G.ALink[next[i].fromTube]] = true;
    }
}

recursiveWalkA(store,next,tube,G,seenTubes,openTubes,dMax);
bool finished = false;
bool same;
int i = 0;
double l1, l2;



while (!finished && seenTubes.size()==0) {
    for (int j = 0; j<store.size();j++) {
        if (j>=store.size()) {
            break;
        } else {
            if ((G.ALink[store[i].index]==G.ALink[store[j].index]) && (i!=j)) {
                l1 = store[i].path.size()-1; // CHECKEN -1 of -2
                l2 = store[j].path.size()-1;
                // length(store(j).path(1:end-1));
                if (l1==l2) {
                    same = true;
                    for (int z = 0; z<l1;z++) { 
                        if (store[i].path[z]!=store[j].path[z]) {
                            same = false;
                        }
                    }
                    if (same) {
                        store.erase(store.begin() + j); // CHECKEN
                    }
                }
            }
        }
    }
    i = i+1;
    if (i>=store.size()) {
        finished = true;
    }
}

}

void vectorFieldMap::validateHypotheses(double x, double y, double vx, double vy) {
    double u, v, likelihood, lTot;
    vector<double> pTemp;
    lTot = 0.0;
    for (int i = 0; i<hypotheses.hypotheses.size(); i++) {
        calcGradient(globalTube.tube[hypotheses.hypotheses[i].path[0]].main, x, y, u, v);
        getLikelihood(u, v, vx, vy, -1.0, 1.0, likelihood);
        pTemp.push_back(likelihood);
        lTot = lTot + likelihood;
    }
    for (int i = 0; i<pTemp.size(); i++) {
        hypotheses.hypotheses[i].p = pTemp[i]/lTot;
        for (int j = 0; j<hypotheses.hypotheses[i].path.size()-1;j++) {
            plotTube(globalTube.tube[hypotheses.hypotheses[i].path[j]].main,20,2,j+i*100,0,0,0.0,0.0,1.0,hypotheses.hypotheses[i].p,"dyamicMap",dynamicMap);
        }
    }


    
}

void vectorFieldMap::updateHypotheses(double x, double y, double vx, double vy) {

    dynamicMap.markers.clear();
    bool consider = considerHuman(human,robot);
    if (consider) {
        vector<bool> seenTubes;
        for (int i=0;i<G.AShort.size();i++) {
            seenTubes.push_back(false);
        }

        vector<recursiveWalkStore> store;
        vector<bool> openTubes;
        human_walking_detection::hypothesis hypothesisTemp;


        createDynamicAsOI(globalTube,robot.x,robot.y,G,seenTubes,openTubes,robot.dMax,store);
        openTubes.clear();
        store.clear();
        createDynamicAsOI(globalTube,x,y,G,openTubes,seenTubes,human.dMax,store);
        hypotheses.hypotheses.clear();
        for (int i = 0; i< store.size();i++) {
            hypothesisTemp.path = store[i].path;
            hypothesisTemp.pStore = store[i].pStore;
            hypothesisTemp.index = store[i].index;
            hypothesisTemp.dTot = store[i].dTot;
            hypotheses.hypotheses.push_back(hypothesisTemp);
        }
        validateHypotheses(x, y, vx, vy);
    }
}

void vectorFieldMap::initializeMap() {
    cout<<"initialize map..."<< endl;
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
    robot.dMax = 3.0;

    human.x = 6.5;
    human.y = 8.2;
    human.r = 3.0;
    human.dMax = 4.0;

    TBC.i = -1;
    
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
        plotTube(globalTube.tube[i].main,20,2,5*i,0,0,1.0,1.0,1.0,0.8,"staticMap",staticMap);
    }
    createGraph(globalTube, G);
    createGraphShort(globalTube, G);

    // cout<<endl;
        // cout<<staticMap.markers.size()<<endl;

        // [store,~] = createDynamicAsOI(globalTube,human,G,[],seenTubes,person.dMax);

        // for (int i = 0; i<store.size();i++) { 
        //     for (int j = 0;j<10;j++) {
        //         z = tube{store(i).index}.min+j*(tube{store(i).index}.max-tube{store(i).index}.min)/10.0;
        //         O = Fry(tube{store(i).index},store(i).pStore,z,0);
        //         plot(O.x,O.y,'.g');
        //     }
        // }

        // tubesH = createTubeHypothesis(tube,store,G,robot);


        // plotAll(1:length(tubesH),1:3,tube,tubesH,store,person,robot)

}