#include <functionsDiscretizedMap.h>
#include <functions.h>

void vectorFieldMap::pointsToLine(hip_msgs::lines &lines) {
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

void vectorFieldMap::intersectLines(hip_msgs::lines lines, positionVars &result) {
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

void vectorFieldMap::initializeTubeStraight(hip_msgs::singleTube &tube) {
hip_msgs::lines lines; 
lines.line = tube.lines;
double norm1 = sqrt(pow(lines.line[0].a,2.0) + pow(lines.line[0].b,2.0));
double norm2 = sqrt(pow(lines.line[1].a,2.0) + pow(lines.line[1].b,2.0));
double signA1 = sign(lines.line[0].a);
double signB1 = sign(lines.line[0].b);
double normVec1 = signA1*(fabs(lines.line[0].a/norm1)+fabs(lines.line[1].a/norm2))/2.0;
double normVec2 = signB1*(fabs(lines.line[0].b/norm1)+fabs(lines.line[1].b/norm2))/2.0;
hip_msgs::transform transform;
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

tube.max = -tube.y1[min1];
tube.min = -tube.y1[max1];

tube.lines = lines.line;
} else {
    tube.empty = true;
}
}

void vectorFieldMap::initializeTubeCurved(hip_msgs::singleTube &tube) {
hip_msgs::lines lines; 
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
if (fabs(tube.theta_tot)>0.1) {
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

void vectorFieldMap::zVal(hip_msgs::singleTube &tube,double x,double y,bool rel,double &z, double &p, bool &inside) {
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
    if (tube.signlr>0.0 && (THETA>0.0 || THETA<tube.theta_tot)) {
        inside = false;
    } else if (tube.signlr<0.0 && (THETA<0.0 || THETA>tube.theta_tot)) {
        inside = false;
    }
    
} else {
    double xT = cos(-tube.transform.theta)*(x-tube.transform.x) - sin(-tube.transform.theta)*(y-tube.transform.y);
    double yT = sin(-tube.transform.theta)*(x-tube.transform.x) + cos(-tube.transform.theta)*(y-tube.transform.y);
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

hip_msgs::line vectorFieldMap::createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2) {
    hip_msgs::line lineTemp;
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

vectorFieldMap::positionVars vectorFieldMap::Fry(hip_msgs::singleTube tube,double I1,double I2,bool rel) {
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

void vectorFieldMap::plotStandingTube(double x, double y, double radius, double r, double g, double b, double a, string ns,
                                        visualization_msgs::MarkerArray &map, string mapFrame, double lifetime) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = mapFrame;
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
    sample.z = 0.0;
    for (int i = 0; i<21;i++) {
        sample.x = radius * cos(2.0*M_PI/20.0*i)+x;
        sample.y = radius * sin(2.0*M_PI/20.0*i)+y;
        marker.points.push_back(sample);

    }   
    marker.id = map.markers.size();
    marker.lifetime = ros::Duration(lifetime);
    map.markers.push_back(marker);
    marker.points.clear();
}

void vectorFieldMap::plotTube(hip_msgs::singleTube tube, int n1, int n2,int subID, bool rel, bool sub,double r, double g, double b, double a, string ns,visualization_msgs::MarkerArray &map, double markerLifetime) {
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
            // linspace(0.0,tube.min,n2/2,plotVar);

        } else {
            linspace(tube.min,tube.max,n2,plotVar);
        }
    } else {
        linspace(tube.min,tube.max,n2,plotVar);
    }    

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/semanticMap";
    //marker.header.frame_id = "/map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    // marker.type = visualization_msgs::Marker::POINTS;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.scale.x = 0.03;
    // marker.scale.x = 0.1;
    // marker.scale.y = 0.1;
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

    if(markerLifetime > 0)
    {
        marker.lifetime = ros::Duration(markerLifetime);
    }
    map.markers.push_back(marker);
    marker.points.clear();
    }
} 

void vectorFieldMap::plotLine(  double x1, double x2, double y1, double y2, 
                                int i, double r, double g, double b, double a, 
                                string ns, string frame, visualization_msgs::MarkerArray &map, double markerLifetime) {
    visualization_msgs::Marker marker;
    //marker.header.frame_id = "/semanticMap";
    marker.header.frame_id = frame;
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
    sample.x  = x1;
    sample.y = y1;
    sample.z = 0.0;
    marker.points.push_back(sample);
    sample.x  = x2;
    sample.y = y2;
    sample.z = 0.0;
    marker.points.push_back(sample);
    marker.lifetime = ros::Duration(markerLifetime);
    
    marker.id = i;
    map.markers.push_back(marker);
    marker.points.clear();
} 

void vectorFieldMap::plotAOI(hip_msgs::singleTube tube, int index, int n, int subID, double a, string ns,double p,visualization_msgs::MarkerArray &map) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/semanticMap";
    //marker.header.frame_id = "/map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time();
    
    // marker.type = visualization_msgs::Marker::POINTS;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color.a = a;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.1;
    // marker.scale.y = 0.2;
    // marker.scale.z = 0.1;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::Point sample;

    vectorFieldMap::positionVars O;
    double zN;
    double xSum = 0.0, ySum = 0.0;
    for (int j = 0; j<n+1; j++) {
        zN = tube.min+double(j)*(tube.max-tube.min)/double(n);
        O = Fry(tube,p,zN,0);
        sample.x  = O.x;
        sample.y = O.y;
        sample.z = 0.0;
        marker.points.push_back(sample);

        if (j != n)
        {
            xSum += O.x;
            ySum += O.y;
        }
    }
    marker.id = subID;
    map.markers.push_back(marker);

    // Add marker ID as well for visualization purposes
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.pose.position.x = xSum/n;
    marker.pose.position.y = ySum/n;
    marker.pose.position.z = 0.0;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.ns = ns + "_IDs";
    marker.text = std::to_string(index);
    marker.scale.z = 0.5;
    map.markers.push_back(marker);

    marker.points.clear();
} 

void vectorFieldMap::linesToTube(hip_msgs::lines &lines, hip_msgs::tubes &tubes, double id, TBCS TBC, bool marked) { 
    // funtion which creates the walking routes
    // lijn aanmaken met createlinetemp -> gradienten op 0 zetten
    // 2 lijnen in lineslist zetten en in deze functie gooien en dan zou ie aangemaakt moeten zijn,
    // functionsDiscretizedmap -> deze funtie helemaal onderaan aanroepen

    // TBC -> tbc.i naar -1 zetten.  Heeft te maken met het opknippen van de tube in componenten

    struct positionVars result;
    hip_msgs::lines linesTemp;
    hip_msgs::singleTube tubeTemp;
    hip_msgs::tube tube;

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
            tubeTemp.marked = marked;
            tubeTemp.plotZ = 101.0;
            tube.main = tubeTemp;
            tubes.tube.push_back(tube); 
        } else {
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
            }
        }
    }
    tubeTemp = {};
    linesTemp.line.clear();
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

void vectorFieldMap::FuvStraight(double dphidx,double dphidy,hip_msgs::singleTube tube,double &u, double &v) {
    double uT = -dphidy;
    double vT = dphidx;
    u = cos(tube.transform.theta)*uT - sin(tube.transform.theta)*vT;
    v = sin(tube.transform.theta)*uT + cos(tube.transform.theta)*vT;
    double l = sqrt(pow(u,2.0)+pow(v,2.0));
    u=u/l;
    v=v/l;
}

void vectorFieldMap::derivativeF(hip_msgs::singleTube tube, double x, double y, double &dphidx, double &dphidy) {
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
dphidx = ((3.0 *c_21 *pow(x,2.0) + 2.0 *c_22 *x + c_23) *(c_4 *y + c_2) + c_3 *(c_12 *x *(c_2 *(c_21 *pow(x,3.0) - c_23 *x) + 2.0 *y) - c_11 *pow(x,2.0) *(c_2 *x *(c_22 *x + 2.0 *c_23) - 3.0 *y) + c_13 *(c_2 *pow(x,2.0) *(2.0 *c_21 *x + c_22) + y)) + c_1 *(c_12 *x *(c_4 *(c_23 *x - c_21 *pow(x,3.0)) + 2.0) + c_11 *pow(x,2.0) *(c_4 *x *(c_22 *x + 2.0 *c_23) + 3.0) - c_13 *(c_4 *pow(x,2.0) *(2.0 *c_21 *x + c_22) - 1.0)))/pow(c_3 *x *(c_11 *pow(x,2.0) + c_12 *x + c_13) + c_4 *x *(c_21 *pow(x,2.0) + c_22 *x + c_23) + 1.0,2.0);
dphidy = -1.0/(c_3 *x *(c_11 *pow(x,2.0) + c_12 *x + c_13) + c_4 *x *(c_21 *pow(x,2.0) + c_22 *x + c_23) + 1.0);
}

void vectorFieldMap::calcGradient(hip_msgs::singleTube tube,double x, double y, double &u, double &v) {
    double z,p,dphidtheta,dphidr,dphidx,dphidy;
    bool inside;
    zVal(tube,x,y,false,z,p,inside);
    if (tube.curved) {
        derivativeF(tube,tube.THETA,tube.r,dphidtheta,dphidr);
        FuvCurved(dphidr,dphidtheta,tube.theta,tube.r,tube.signlr,u,v);
    } else {
        double xT = cos(-tube.transform.theta)*(x-tube.transform.x) - sin(-tube.transform.theta)*(y-tube.transform.y); 

        double yT = sin(-tube.transform.theta)*(x-tube.transform.x) + cos(-tube.transform.theta)*(y-tube.transform.y); 

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

void vectorFieldMap::findPointInTube(hip_msgs::tubes tubes, double x, double y, vector<int> &index) {
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

void vectorFieldMap::validatePointInSubTube(hip_msgs::tube tube, double x, double y, double vx, double vy,vector<double> &prop) {
double z,p,pTot;
bool inside;
double xTemp, yTemp, Dneg, Dpos,C, u, v, likelihood;
vectorFieldMap::positionVars Omax,Omin;
prop.clear();
for (int i = 0; i<3; i++) {
    prop.push_back(0.0);
}
pTot = 0;
int counter = 0;
for (int ind = 0;ind<tube.subTubes0.size();ind++) {
    zVal(tube.subTubes0[ind],x,y,false,z,p,inside);
    if (inside) {
        calcGradient(tube.subTubes0[ind], x, y, u, v); 

        Omax = Fry(tube.subTubes0[ind],p,tube.subTubes0[ind].max,0);
        Omin = Fry(tube.subTubes0[ind],p,tube.subTubes0[ind].min,0);
        if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
            Dpos =  dist(x,y,Omin.x,Omin.y);
            Dneg = -dist(x,y,Omax.x,Omax.y);
        } else {
            Dneg = -dist(x,y,Omin.x,Omin.y);
            Dpos =  dist(x,y,Omax.x,Omax.y);
        }
        getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
        prop[0] = prop[0] + likelihood;
        pTot = pTot + likelihood;
        counter++;
    }
}
for (int ind = 0;ind<tube.subTubesB0.size();ind++) {
    zVal(tube.subTubesB0[ind],x,y,false,z,p,inside);
    if (inside) {
        calcGradient(tube.subTubesB0[ind], x, y, u, v); 

        Omax = Fry(tube.subTubesB0[ind],p,tube.subTubesB0[ind].max,0);
        Omin = Fry(tube.subTubesB0[ind],p,tube.subTubesB0[ind].min,0);
        if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
            Dpos =  dist(x,y,Omin.x,Omin.y);
            Dneg = -dist(x,y,Omax.x,Omax.y);
        } else {
            Dneg = -dist(x,y,Omin.x,Omin.y);
            Dpos =  dist(x,y,Omax.x,Omax.y);
        }
        getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
        prop[0] = prop[0] + likelihood;
        pTot = pTot + likelihood;
        counter++;
    }
}
for (int ind = 0;ind<tube.subTubes1.size();ind++) {
    zVal(tube.subTubes1[ind],x,y,false,z,p,inside);
    if (inside) {
        calcGradient(tube.subTubes1[ind], x, y, u, v); 

        Omax = Fry(tube.subTubes1[ind],p,tube.subTubes1[ind].max,0);
        Omin = Fry(tube.subTubes1[ind],p,tube.subTubes1[ind].min,0);
        if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
            Dpos =  dist(x,y,Omin.x,Omin.y);
            Dneg = -dist(x,y,Omax.x,Omax.y);
        } else {
            Dneg = -dist(x,y,Omin.x,Omin.y);
            Dpos =  dist(x,y,Omax.x,Omax.y);
        }
        getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
        prop[1] = prop[1] + likelihood;
        pTot = pTot + likelihood;
        counter++;
    }
}
for (int ind = 0;ind<tube.subTubesB1.size();ind++) {
    zVal(tube.subTubesB1[ind],x,y,false,z,p,inside);
    if (inside) {
        calcGradient(tube.subTubesB1[ind], x, y, u, v); 
        Omax = Fry(tube.subTubesB1[ind],p,tube.subTubesB1[ind].max,0);
        Omin = Fry(tube.subTubesB1[ind],p,tube.subTubesB1[ind].min,0);
        if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
            Dpos =  dist(x,y,Omin.x,Omin.y);
            Dneg = -dist(x,y,Omax.x,Omax.y);
        } else {
            Dneg = -dist(x,y,Omin.x,Omin.y);
            Dpos =  dist(x,y,Omax.x,Omax.y);
        }
        getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
        prop[1] = prop[1] + likelihood;
        pTot = pTot + likelihood;
        counter++;
    }
}
for (int ind = 0;ind<tube.subTubes2.size();ind++) {
    if (tube.subTubesB2.size()>ind) {
        zVal(tube.subTubes2[ind],x,y,false,z,p,inside);
        if (inside) {
            calcGradient(tube.subTubes2[ind], x, y, u, v); 
            Omax = Fry(tube.subTubes2[ind],p,tube.subTubes2[ind].plotZ,0);
            zVal(tube.subTubesB2[ind],x,y,false,z,p,inside);
            Omin = Fry(tube.subTubesB2[ind],p,tube.subTubesB2[ind].plotZ,0);
            if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
                Dpos =  dist(x,y,Omin.x,Omin.y);
                Dneg = -dist(x,y,Omax.x,Omax.y);
            } else {
                Dneg = -dist(x,y,Omin.x,Omin.y);
                Dpos =  dist(x,y,Omax.x,Omax.y);
            }
            getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
            prop[2] = prop[2] + likelihood;
        } else {
            zVal(tube.subTubesB2[ind],x,y,false,z,p,inside);
            if (inside) {
                calcGradient(tube.subTubesB2[ind], x, y, u, v); 
                Omax = Fry(tube.subTubesB2[ind],p,tube.subTubesB2[ind].plotZ,0);
                zVal(tube.subTubes2[ind],x,y,false,z,p,inside);
                Omin = Fry(tube.subTubes2[ind],p,tube.subTubes2[ind].plotZ,0);
                if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
                    Dpos =  dist(x,y,Omin.x,Omin.y);
                    Dneg = -dist(x,y,Omax.x,Omax.y);
                } else {
                    Dneg = -dist(x,y,Omin.x,Omin.y);
                    Dpos =  dist(x,y,Omax.x,Omax.y);
                }
                getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
                prop[2] = prop[2] + likelihood;
            }
        }
    }
}
}

void vectorFieldMap::createGraph(hip_msgs::tubes tube, graph &G) {
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
        for (int pInd = 0;pInd<2;pInd++) {
            p = pVars[pInd];
            O = Fry(tube.tube[i].main,p,(tube.tube[i].main.min+tube.tube[i].main.max)/2.0,false);
            calcGradient(tube.tube[i].main,O.x,O.y,u,v);
            for (int sInd = 0;sInd<2;sInd++) {
                s = sVars[sInd];
                findPointInTube(tube,O.x+u*s,O.y+v*s,index);
                valid = true;
                for (int j = 0;j<index.size();j++) {
                    zVal(tube.tube[index[j]].main,O.x+u*s,O.y+v*s,0,z,p,inside);
                    
                    if ((fabs(p-tube.tube[index[j]].main.pTot)>0.05) && (fabs(p)>0.05)) {
                        index[j] = -1;
                    }
                    if (index[j]==i) {
                        valid = false;
                    }
                }
                if (valid) {
                    for (int j = 0;j<index.size();j++) {
                        if (index[j]!=-1) {
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

void vectorFieldMap::createGraphShort(hip_msgs::tubes tube, graph &G) {
vector<bool> visited;
struct positionVars O;
for (int i=0;i<G.A.size();i++) {
    visited.push_back(false);
}
vector<int> coupledTemp,ALink;
vector<vector<int>> coupled;
vector<int> index;
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
}

bool vectorFieldMap::considerHuman(hip_msgs::human person,hip_msgs::robot robot) {
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

void vectorFieldMap::distEnd(double x,double y,vector<recursiveWalkStore> &store,vector<recursiveWalk> prev,vector<recursiveWalk> &next,hip_msgs::tubes tube,graph G,vector<bool> openTubes,double dMax) {
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

void vectorFieldMap::recursiveWalkA(vector<recursiveWalkStore> &store,vector<recursiveWalk> next,hip_msgs::tubes tube,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax) {
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

void vectorFieldMap::createDynamicAsOI(hip_msgs::tubes tube,double xPR, double yPR,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax,vector<recursiveWalkStore> &store) {
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
                l1 = store[i].path.size()-1; 
                l2 = store[j].path.size()-1;
                if (l1==l2) {
                    same = true;
                    for (int z = 0; z<l1;z++) { 
                        if (store[i].path[z]!=store[j].path[z]) {
                            same = false;
                        }
                    }
                    if (same) {
                        store.erase(store.begin() + j); 
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

void vectorFieldMap::borderTransform(hip_msgs::tubes &tube) {
double p, u, v;
vectorFieldMap::positionVars O, OMax, OMin, O1, O2, O_old;

if (tube.tube[0].main.dir==1) {
	p = 0.0;
} else {
	p = tube.tube[0].main.pTot;
}

O = Fry(tube.tube[0].main,p,(tube.tube[0].main.max+tube.tube[0].main.min)/2.0,0);
calcGradient(tube.tube[0].main,O.x,O.y,u,v);
double uT, vT, testVar, Z1, Z2;
uT = v;
vT = -u;
OMax = Fry(tube.tube[0].main,p,tube.tube[0].main.max,0);
OMin = Fry(tube.tube[0].main,p,tube.tube[0].main.min,0);
testVar = (OMin.x-OMax.x)*uT + (OMin.y-OMax.y)*vT;
if (testVar>0) {
    Z1 = tube.tube[0].main.max;
    Z2 = tube.tube[0].main.min;
} else {
    Z1 = tube.tube[0].main.min;
    Z2 = tube.tube[0].main.max;
}
double transTempC1, transTempC2;

transTempC1 = -Z1;
transTempC2 = 1.0/(Z2-Z1);
tube.tube[0].main.cTransC1 = transTempC1;
tube.tube[0].main.cTransC2 = transTempC2;

bool inside;
double PUNUSED;
for (int i = 1; i<tube.tube.size(); i++) {
    if (tube.tube[i-1].main.dir==1) {
    	p = tube.tube[i-1].main.pTot;
    } else {
    	p = 0.0;
    }
    O1 = Fry(tube.tube[i-1].main,p,0.0,1);
    O2 = Fry(tube.tube[i-1].main,p,1.0,1);
    
    zVal(tube.tube[i].main,O1.x,O1.y,0,Z1,PUNUSED,inside);
    zVal(tube.tube[i].main,O2.x,O2.y,0,Z2,PUNUSED,inside);
    // Z2 = zVal(tube{i},O2.x,O2.y,0);
    tube.tube[i].main.cTransC1 = -Z1;
    tube.tube[i].main.cTransC2 = 1.0/(Z2-Z1);;

}
    

vector<double> dTubeCum, prog, dTube;

for (int z=0; z<5; z++) {
    dTubeCum.push_back(0.0);
}
// prog = [];
double P, d;

for (int i=0; i< tube.tube.size(); i++) {
    if (tube.tube[i].main.curved) {
        P = tube.tube[i].main.theta_tot;
    } else {
        P = tube.tube[i].main.dx;
    }
    prog.push_back(P);
    // dTube = [];
    for (int z=0; z<5; z++) {
        O_old = Fry(tube.tube[i].main,P/100.0*0.0,z/4.0,1);
        d=0.0;
        for (int j=1; j<101; j++) {
            O = Fry(tube.tube[i].main,P/100.0*j,z/4.0,1);
            d = d+ dist(O.x,O.y, O_old.x, O_old.y);
            // sqrt((O.x-O_old.x)^2+(O.y-O_old.y)^2);
            O_old = O;
        }
        dTube.push_back(d);
    }
    tube.tube[i].main.dTube = dTube;
    for (int z = 0; z<5; z++) {
        dTubeCum[z] = dTubeCum[z] + dTube[z];
    }
    tube.tube[i].main.dTubeCum = dTubeCum;
}
double totProg = 0.0;
for (int i=0; i<prog.size(); i++) {
    totProg = totProg + fabs(prog[i]);
}


// totProg = sum(abs(prog));
double pCum_old = 0.0;
double A11, A12, A21, A22, b1, b2, pCum, idet, pStart, pEnd;
for (int i = 0; i<tube.tube.size(); i++) {
    if (tube.tube[i].main.dir==1) {
        pStart = 0.0;
        pEnd = tube.tube[i].main.pTot;
    } else {
        pStart = tube.tube[i].main.pTot;
        pEnd = 0.0;
    }
    A11 = pStart;
    A12 = 1.0;
    A21 = pEnd;
    A22 = 1.0;
    pCum = pCum_old + fabs(tube.tube[i].main.pTot)/totProg;
    b1 = pCum_old;
    b2 = pCum;
    pCum_old = pCum;
    idet = 1.0/(A11*A22-A12*A21);
    tube.tube[i].main.pM = idet*A22*b1 - idet*A12*b2;
    tube.tube[i].main.pC = -idet*A21*b1 + idet*A11*b2;
}

}

void vectorFieldMap::walkConstant(hip_msgs::singleTube tube,double ds,double &x,double &y, double &C, double &p, double &D) {
    D = 0.0;
    double step = 0.01;
    bool inside;
    zVal(tube,x,y,true,C,p,inside);
    vectorFieldMap::positionVars O;

    if (!inside) {
        x = -1; 
        y = -1;
    } else {
        double s = sign(ds);
        if (ds==0) {
            O.x = x;
            O.y = y;
        } else {
            O = Fry(tube,p,C,1);
            D = dist(O.x,O.y,x,y);
            while (D<fabs(ds) && inside) {
                O = Fry(tube,p,C,1);
                D = dist(O.x,O.y,x,y);
                C = C + s * step;
                if (C <= 0 || C >= 1) {
                    inside = false;
                }
            }
        }
        x = O.x;
        y = O.y;
    }
}

void vectorFieldMap::validateHypotheses(double x, double y, double vx, double vy, vector<hip_msgs::tubes> tubesH) {
    double u, v, likelihood, lTot;
    bool inside;
    vector<double> pTemp;
    lTot = 0.0;
    vectorFieldMap::positionVars O, O_old, Omax,Omin;
    double DUMMYDOUBLE;
    bool DUMMYBOOL;
    double xTemp, yTemp, C, Dpos, Dneg, p;
    vector<double> propSub;
    hip_msgs::hypothesis hypothesisTemp;
    lTot = 0.3;//lTot + 0.5;

    double vAbs = sqrt(pow(vx,2.0)+pow(vy,2.0));
    if (vAbs<0.1) {
        likelihood = -10.0*vAbs+1.0;
    } else {
        likelihood = 0.0;
    }
    lTot = lTot + likelihood;
    hypothesisTemp.p = likelihood;
    hypotheses.hypotheses.push_back(hypothesisTemp);
    // cout<<hypotheses.hypotheses.size();

    for (int i = 0; i<hypotheses.hypotheses.size()-1; i++) {
        // calcGradient(globalTube.tube[hypotheses.hypotheses[i].path[0]].main, x, y, u, v);
        xTemp = x;
        yTemp = y;

        if (tubesH[i].tube[0].subTubes0.size()>0) {
            validatePointInSubTube(tubesH[i].tube[0], x, y, vx, vy,propSub);
            for (int j = 0; j<propSub.size(); j++) {
                pTemp.push_back(propSub[j]);
                lTot = lTot + propSub[j];
            }
        } else {
            calcGradient(tubesH[i].tube[0].main, x, y, u, v); 
            zVal(tubesH[i].tube[0].main,x,y,0,DUMMYDOUBLE,p,DUMMYBOOL);
            Omax = Fry(tubesH[i].tube[0].main,p,tubesH[i].tube[0].main.max,0);
            Omin = Fry(tubesH[i].tube[0].main,p,tubesH[i].tube[0].main.min,0);
            if (dist(x,y,Omin.x,Omin.y)<dist(x-v/10.0,y+u/10.0,Omin.x,Omin.y)) {
                Dpos =  dist(x,y,Omin.x,Omin.y);
                Dneg = -dist(x,y,Omax.x,Omax.y);
            } else {
                Dneg = -dist(x,y,Omin.x,Omin.y);
                Dpos =  dist(x,y,Omax.x,Omax.y);
            }
            getLikelihood(u, v, vx, vy, Dneg, Dpos, likelihood);
            pTemp.push_back(likelihood);
            lTot = lTot + likelihood;

        }
    }
    // lTot = 1.0;
    int count = 0;
    for (int i = 0; i<hypotheses.hypotheses.size()-1; i++) {
        if (tubesH[i].tube[0].subTubes0.size()>0) {
            hypotheses.hypotheses[i].pSub.clear();
            for (int j = 0; j<3; j++) {
                hypotheses.hypotheses[i].pSub.push_back(pTemp[count]/lTot);
                count++;
            }
            hypotheses.hypotheses[i].p = 0.0;
            hypotheses.hypotheses[i].pAOI = (hypotheses.hypotheses[i].pSub[0]+hypotheses.hypotheses[i].pSub[1]+hypotheses.hypotheses[i].pSub[2]);
        } else {
            hypotheses.hypotheses[i].p = pTemp[count]/lTot;
            hypotheses.hypotheses[i].pAOI = pTemp[count]/lTot;
            count++;
            hypotheses.hypotheses[i].pSub.clear();
            for (int j = 0; j<3; j++) {
                hypotheses.hypotheses[i].pSub.push_back(0.0);
            }
        }
    }
}

void vectorFieldMap::createTubeHypothesis(hip_msgs::tubes tube,vector<recursiveWalkStore> store,graph G,hip_msgs::robot robot, vector<hip_msgs::tubes> &tubesH, std::vector<KalmanFilter> humanFilters, int humanConsidered) {
hip_msgs::tubes tubeTemp; // bevat deel tubes (bv, bocht, kruising)
// dir -> direction which can be +1 or -1
//store: alle mogelijke paden die iemand kan lopen
// store[i].path[j] -> componenten van pad
for (int i = 0; i<store.size(); i++) {
    for (int j = 0; j<store[i].path.size()-1; j++) {
        tubeTemp.tube.push_back(tube.tube[store[i].path[j]]);
        tubeTemp.tube[j].main.dir = G.A[store[i].path[j+1]][store[i].path[j]];
    }

    borderTransform(tubeTemp); // bovenkant en onderkant (stroomlijn) van de tube 0 en 1. Maakt grenzen van het veld 0 en 1, constante waarde geeft stroomlijn
    addObject(tubeTemp,robot,0); // linksom
    addObject(tubeTemp,robot,1); // rechtsom
    addObject(tubeTemp,robot,2);  // tegen robot aan (wordt toegevoegd in tubetemp)
    tubesH.push_back(tubeTemp); // zet hier een extra
    tubeTemp.tube.clear();
}

for(int i = 0; i < humanFilters.size(); i++)
{
    if(i == humanConsidered)
    {
        continue;
    }

    hip_msgs::PoseVel thisHuman = humanFilters[humanConsidered].predictPos(humanFilters[humanConsidered].getLatestUpdateTime() ); // TODO propagate to now?
    hip_msgs::PoseVel otherHuman = humanFilters[i].predictPos(humanFilters[i].getLatestUpdateTime() );

// tubeTemp per persoon hier een tube toevoegen zoals in regel 530 beschreven
//tubeTemp vullen met alle benodige onderdelen

// tubeTemp.tube[j].main.dir ook toevoegen
// lijnen definieren in de loodrechtrichting

    TBCS TBC;
    TBC.i = -1;

    // perpendicular direction
    double dx = otherHuman.x - thisHuman.x;
    double dy = otherHuman.y - thisHuman.y;
    double vectorLength = sqrt( pow(dx, 2.0) + pow(dy, 2.0) );
    double width = 0.5; // [m]

    double dxPerpendicular = -0.5*width/vectorLength*dy;
    double dyPerpendicular =  0.5*width/vectorLength*dx;
    
    hip_msgs::lines lines;
    hip_msgs::line lineTemp;
    lineTemp = createLineTemp(thisHuman.x + dxPerpendicular, thisHuman.x - dxPerpendicular, thisHuman.y + dyPerpendicular, thisHuman.y - dyPerpendicular, 0.0, 0.0);
    lines.line.push_back(lineTemp);

    // createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2)

    lineTemp = createLineTemp(otherHuman.x + dxPerpendicular, otherHuman.x - dxPerpendicular, otherHuman.y + dyPerpendicular, otherHuman.y - dyPerpendicular,0.0, 0.0);
    lines.line.push_back(lineTemp);
    linesToTube(lines, tubeTemp, 0, TBC, 0);

//    marked ??
//    id
    //linesToTube(hip_msgs::lines &lines, hip_msgs::tubes &tubes, double id, TBCS TBC, bool marked)

    for(int j=0; j < tubeTemp.tube.size(); j++)
    {
        tubeTemp.tube[j].main.dir = 1;
    }
    

    createGraph(tubeTemp, G); // Niet zeker of deze 2 regels nodig zijn
    createGraphShort(tubeTemp, G); // Niet zeker of deze 2 regels nodig zijn
//    borderTransform(tubeTemp); // required?
    tubesH.push_back(tubeTemp);
}

}

void vectorFieldMap::updateHypotheses(//double x, double y, double vx, double vy, 
std::vector<KalmanFilter> humanFilters, int humanConsidered, double xRobot, double yRobot, double thetaRobot, string robotFrame, string mapFrame, double markerLifetime) {

    // give list of all persons and id of person which is considered atm to derive human position

    robot.x = xRobot;
    robot.y = yRobot;
    robot.theta = thetaRobot;
    hip_msgs::PoseVel thisHuman = humanFilters[humanConsidered].predictPos(humanFilters[humanConsidered].getLatestUpdateTime() );
//std::cout << "updateHypotheses, humanConsidered = " << humanConsidered << " thisHuman.x = " << thisHuman.x << " thisHuman.y = " << thisHuman.y << std::endl;
    human.x = thisHuman.x;
    human.y = thisHuman.y;
/*    double vx = thisHuman.vx;
    double vy = thisHuman.vy;
*/
    dynamicMap.markers.clear();
    bool consider = considerHuman(human,robot);
    plotRobot(robot, robotFrame, markerLifetime);
    vector<hip_msgs::tubes> tubesH;

    if (consider) 
    {
        vector<bool> seenTubes;
        for (int i=0;i<G.AShort.size();i++) {
            seenTubes.push_back(false);
        }

        vector<recursiveWalkStore> store;
        vector<bool> openTubes;
        hip_msgs::hypothesis hypothesisTemp;
        hip_msgs::hypotheses hypothesesNew;
        hip_msgs::hypotheses hypothesesOld;


        createDynamicAsOI(globalTube,robot.x,robot.y,G,seenTubes,openTubes,robot.dMax,store);
        openTubes.clear();
        store.clear();
        createDynamicAsOI(globalTube,thisHuman.x,thisHuman.y,G,openTubes,seenTubes,human.dMax,store);
        vector<hip_msgs::tubes> tubesHOld;
        bool skip,same;
        vector<int> removeElements;
        tubesHOld.clear();
        hypotheses.hypotheses.clear();

        for (int i = 0; i< store.size();i++) {
            skip = false;

            if (!skip) {
                hypothesisTemp.path = store[i].path;
                hypothesisTemp.pStore = store[i].pStore;
                hypothesisTemp.index = store[i].index;
                hypothesisTemp.dTot = store[i].dTot;
                hypotheses.hypotheses.push_back(hypothesisTemp);
            }
        }



        createTubeHypothesis(globalTube, store, G, robot, tubesH, humanFilters, humanConsidered);
        // cout<<tubesH.size()<<endl;

        int nOtherObjectsOfInterestConsidered = humanFilters.size() - 1; // cause person itself is not a target

        for(int i = 0; i < nOtherObjectsOfInterestConsidered; i++)
        {
            hypotheses.hypotheses.push_back(hypothesisTemp);
        }
        validateHypotheses(thisHuman.x, thisHuman.y, thisHuman.vx, thisHuman.vy, tubesH);

        // cout<<"debug3: "<<tubesH.size()<<endl;

        vector<bool> visitedH;
        // vector<double> p;
        // vector<double> pStore;
        // vector<int> tubeStore;
        for (int i = 0; i< hypotheses.hypotheses.size(); i++) {
            visitedH.push_back(false);
        } 
        double p;
        for (int i = 0; i< hypotheses.hypotheses.size(); i++) {
            if (!visitedH[i]) {
                p = hypotheses.hypotheses[i].pAOI;
                visitedH[i] = true;
                for (int j = 0; j<hypotheses.hypotheses.size(); j++) {
                    if (!visitedH[j]) {
                        if (G.ALink[hypotheses.hypotheses[i].index]==G.ALink[hypotheses.hypotheses[j].index]) {
                            p = p + hypotheses.hypotheses[j].pAOI;
                            visitedH[j] = true;
                        }
                    }
                }
                plotAOI(globalTube.tube[hypotheses.hypotheses[i].index].main, hypotheses.hypotheses[i].index, 10, dynamicMap.markers.size(), p*0.95+0.05, "AoIMap", hypotheses.hypotheses[i].pStore, dynamicMap);
            }
        }

        // cout<<pTotPlot<<endl;
        double pTotPlot=0;
        for (int i=0;i<hypotheses.hypotheses.size();i++) {
            pTotPlot=pTotPlot + hypotheses.hypotheses[i].pAOI;
        }
        pTotPlot=pTotPlot+hypotheses.hypotheses[hypotheses.hypotheses.size()-1].p;
        plotLine(-0.4,-0.4,6.0,6.5-(6.5-2.5)*(1-pTotPlot),dynamicMap.markers.size(),0.5,0,0.5,0.6,"dynamicMapNOTA", "/semanticMap",dynamicMap, markerLifetime); 
        // cout<<"debug4: "<<hypotheses.hypotheses.size()<<", "<<tubesH.size()<<endl;
        globalTubesH.tubeH.clear();
        for (int i = 0; i<tubesH.size();i++) {
            globalTubesH.tubeH.push_back(tubesH[i]);
        }

        for (int i = 0; i<tubesH.size(); i++) {
                for (int j = 0; j<tubesH[i].tube.size();j++) {
                    // if (hypotheses.hypotheses[i].index==15) {
                    plotTube(tubesH[i].tube[j].main,20,2,dynamicMap.markers.size(),0,0,0.0,0.0,1.0,hypotheses.hypotheses[i].p*1.0+0.00,"dynamicMap",dynamicMap, markerLifetime);
                    //plotTube(hip_msgs::singleTube tube, int n1, int n2,int subID, bool rel, bool sub,double r, double g, double b, double a, string ns,visualization_msgs::MarkerArray &map, double markerLifetime) {
                    for (int z = 0; z<tubesH[i].tube[j].subTubes0.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubes0[z],20,2,dynamicMap.markers.size(),0,1,0.0,0.0,1.0,hypotheses.hypotheses[i].pSub[0]*1.0+0.0,"dynamicMap0",dynamicMap, markerLifetime);
                    }
                    for (int z = 0; z<tubesH[i].tube[j].subTubes1.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubes1[z],20,2,dynamicMap.markers.size(),0,1,0.0,0.0,1.0,hypotheses.hypotheses[i].pSub[1]*1.0+0.0,"dynamicMap1",dynamicMap, markerLifetime);
                    }
                    for (int z = 0; z<tubesH[i].tube[j].subTubes2.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubes2[z],20,2,dynamicMap.markers.size(),0,1,1.0,0.0,0.0,hypotheses.hypotheses[i].pSub[2]*1.0+0.0,"dynamicMap2",dynamicMap, markerLifetime);
                    }
                    for (int z = 0; z<tubesH[i].tube[j].subTubesB0.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubesB0[z],20,2,dynamicMap.markers.size(),0,1,0.0,0.0,1.0,hypotheses.hypotheses[i].pSub[0]*1.0+0.0,"dynamicMapB0",dynamicMap, markerLifetime);
                    }
                    for (int z = 0; z<tubesH[i].tube[j].subTubesB1.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubesB1[z],20,2,dynamicMap.markers.size(),0,1,0.0,0.0,1.0,hypotheses.hypotheses[i].pSub[1]*1.0+0.0,"dynamicMapB1",dynamicMap, markerLifetime);
                    }
                    for (int z = 0; z<tubesH[i].tube[j].subTubesB2.size();z++) {
                        plotTube(tubesH[i].tube[j].subTubesB2[z],20,2,dynamicMap.markers.size(),0,1,1.0,0.0,0.0,hypotheses.hypotheses[i].pSub[2]*1.0+0.0,"dynamicMapB2",dynamicMap, markerLifetime);
                    }
                    // }
                }
        }
    
    plotStandingTube(thisHuman.x, thisHuman.y, 0.5, 0.0, 1.0, 0.0, hypotheses.hypotheses[hypotheses.hypotheses.size()-1].p*1.0 + 0.00, "AoIMap", dynamicMap, mapFrame, markerLifetime);

    }

}

int vectorFieldMap::findPGlobalInTube(hip_msgs::tubes tube, double p) {
    int index = -1;
    for (int i = 0; i<tube.tube.size(); i++) {
        if (tube.tube[i].main.pTot*tube.tube[i].main.pM>0.0) {
            if (p<=tube.tube[i].main.pTot*tube.tube[i].main.pM+tube.tube[i].main.pC && p>=tube.tube[i].main.pC) {
                    index = i;
            }
        } else {
            if (p>=tube.tube[i].main.pTot*tube.tube[i].main.pM+tube.tube[i].main.pC && p<=tube.tube[i].main.pC) {
                    index = i;
            }
        }
    }
    return index;
}

void vectorFieldMap::walkToBorder(double &x,double &y,double dMax,hip_msgs::tubes tube,bool &endReached, double &dist, bool &border) {
    border=false;
    double scale;

    if (endReached) {
    scale = 1.0;
    } else {
        scale = -1.0;
    }
    vector<int> iS;
    findPointInTube(tube,x,y,iS);
    hip_msgs::singleTube tubeE;
    vectorFieldMap::positionVars O;
    double z, pS, p;
    bool inside;

    if (iS.size()>0) {
    tubeE = tube.tube[iS[0]].main;
    double p1, p2, pStart, pEnd;
    p1 = tubeE.pC;
    if (tubeE.curved) {
        p2 = tubeE.theta_tot*tubeE.pM + tubeE.pC;
    } else {
        p2 = tubeE.dx*tubeE.pM + tubeE.pC;
    }
    if (p1>p2) {
        pStart = p2-scale*0.0001;
        pEnd = p1+scale*0.0001;
    } else {
        pStart = p1-scale*0.0001;
        pEnd = p2+scale*0.0001;
    }
    double target, targetRel;
    if (dMax>0.0) {
        targetRel = pEnd;
    } else {
        targetRel = pStart;
    }
    target = (targetRel - tubeE.pC)/tubeE.pM;
    zVal(tubeE,x,y,1,z,pS,inside);

    int steps = 40;
    dist = 0.0;
    endReached = 1;
    for (int i = 1; i<steps+1; i++) {
        p = double(i)*(target - pS)/double(steps) + pS;
        O = Fry(tubeE,p,z,1);
        dist = dist + sqrt(pow(O.x-x,2.0)+pow(O.y-y,2.0));
        if (dist>fabs(dMax)) {
            endReached = 0;
            break;
        }
        x = O.x;
        y = O.y;
    }

    double pAbs = p*tubeE.pM+tubeE.pC;
    
    if (pAbs<0.0 || pAbs>1.0) {
        border = true;
        p = double(steps-1)*(target - pS)/double(steps) + pS; 
        O = Fry(tubeE,p,z,1);
        x = O.x;
        y = O.y;
        endReached = false;
    }
    } else { 
        border = true;
        endReached = false;
        dist = 0.0;
    }
}

void vectorFieldMap::minVal(vector<double> prTemp, double &minVal, int &minInd) {
    minVal = prTemp[0];
    minInd = 0;
    for (int i = 1; i<prTemp.size(); i++) {
        if (prTemp[i]<minVal) {
            minVal = prTemp[i];
            minInd = i;
        }
    }
}

void vectorFieldMap::maxVal(vector<double> prTemp, double &maxVal, int &maxInd) {
    maxVal = prTemp[0];
    maxInd = 0;
    for (int i = 1; i<prTemp.size(); i++) {
        if (prTemp[i]>maxVal) {
            maxVal = prTemp[i];
            maxInd = i;
        }
    }
}

double vectorFieldMap::angleBetweenVectors(hip_msgs::line lineTemp,double v1[2]) {
    double v2a[2], v2b[2];
    double theta;
    v2a[0] = -lineTemp.y[1]+lineTemp.y[0];
    v2a[1] =  lineTemp.x[1]-lineTemp.x[0];
    v2b[0] =  lineTemp.y[1]-lineTemp.y[0];
    v2b[1] = -lineTemp.x[1]+lineTemp.x[0];
    if (v1[0]*v2a[0]+v1[1]*v2a[1]>v1[0]*v2b[0]+v1[1]*v2b[1]) {
        theta = asin((v2a[0]*v1[1]-v2a[1]*v1[0])/(sqrt(pow(v1[0],2.0)+pow(v1[1],2.0))*sqrt(pow(v2a[0],2.0)+pow(v2a[1],2.0))));
    } else {
        theta = asin((v2b[0]*v1[1]-v2b[1]*v1[0])/(sqrt(pow(v1[0],2.0)+pow(v1[1],2.0))*sqrt(pow(v2b[0],2.0)+pow(v2b[1],2.0))));
    }
    return theta;
}

double vectorFieldMap::relativeGradientV2(hip_msgs::line lineTemp,hip_msgs::singleTube tube,int i) {
    i = i-1;
    vectorFieldMap::positionVars O;
    double DUMMYDOUBLE;
    bool DUMMYBOOL;
    int j;
    double x, y, z, p, u, v;
    double v1[2];

    if (i==0) {
        j=1;
    } else {
        j=0;
    }

    zVal(tube,lineTemp.x[i],lineTemp.y[i],true,z,DUMMYDOUBLE,DUMMYBOOL);
    x = lineTemp.x[i];
    y = lineTemp.y[i];
    if (z<0.01) {
        zVal(tube,lineTemp.x[j],lineTemp.y[j],1,DUMMYDOUBLE,p,DUMMYBOOL);
        O = Fry(tube,p,0.01,1);
        x = O.x;
        y = O.y;
    } else if (z>0.99) {
        zVal(tube,lineTemp.x[j],lineTemp.y[j],1,DUMMYDOUBLE,p,DUMMYBOOL);
        O = Fry(tube,p,0.99,1);
        x = O.x;
        y = O.y;
    }

    calcGradient(tube,x,y,u,v);
    v1[0] = u;
    v1[1] = v;

    return angleBetweenVectors(lineTemp,v1);
}

void vectorFieldMap::addBorderElement(double zFixed,double zBorder,hip_msgs::tubes tube,double p,double dfStart,hip_msgs::lines linesB,double index,positionVars O,double theta_2) {
    struct positionVars OB;
    hip_msgs::line lineTemp;
    if (fabs(zFixed-zBorder)>0.01) {
        OB = Fry(tube.tube[index].main,p,zBorder,1);
        lineTemp = createLineTemp(OB.x,O.x,OB.y,O.y,0.0,0.0);
        lineTemp = createLineTemp(OB.x,O.x,OB.y,O.y,relativeGradientV2(lineTemp,tube.tube[index].main,1),theta_2);
        lineTemp.ind = index;
        if (dfStart<0) {
            linesB.line.insert(linesB.line.begin(), lineTemp);
        } else {
            linesB.line.push_back(lineTemp);
        }
    }
}

void vectorFieldMap::splitFrontBack(double &xLoop,double &yLoop,double &df,hip_msgs::tubes tube,bool &endReached,hip_msgs::lines &lines,hip_msgs::lines &linesB,double &dTot,double zVar,vector<double> CfT1,vector<double> CfT2,double zFixed,double zBorder,double dfStart) {
    double DUMMYDOUBLE;
    bool DUMMYBOOL;
    vectorFieldMap::positionVars O, O_old;
    bool notLastElement = true;
    if (!endReached) {
        notLastElement = false;
    }
    bool border;
    double dBorder;
    walkToBorder(xLoop,yLoop,df,tube,endReached, dBorder, border);

    dTot = dTot + dBorder;
    O_old.x = xLoop;
    O_old.y = yLoop;
    df = df - sign(df)*dBorder;
    double ds1, ds2, p;
    ds1 = pow(dTot,3.0) * CfT1[0] +  pow(dTot,2.0) * CfT1[1] +  dTot * CfT1[2];
    ds2 = pow(dTot,3.0) * CfT2[0] +  pow(dTot,2.0) * CfT2[1] +  dTot * CfT2[2];
    vector<int> indexVec;
    findPointInTube(tube,xLoop,yLoop,indexVec);
    if (indexVec.size()>0) {
    double index = indexVec[0];
    zVal(tube.tube[index].main,xLoop,yLoop,1,DUMMYDOUBLE,p,DUMMYBOOL);
    O = Fry(tube.tube[index].main,p,zVar+ds1,1);

    xLoop = O.x;
    yLoop = O.y;
    O = Fry(tube.tube[index].main,p,zFixed+ds2,1);
    hip_msgs::line lineTemp;
    lineTemp = createLineTemp(xLoop,O.x,yLoop,O.y,0.0,0.0); 
    double theta_1, theta_2,dAbsolute1,dAbsolute2;   
    theta_1 = relativeGradientV2(lineTemp,tube.tube[index].main,1);
    theta_2 = relativeGradientV2(lineTemp,tube.tube[index].main,2);

    if (notLastElement) {
        dAbsolute1 = dist(O_old.x,O_old.y,xLoop,yLoop)/fabs(ds1);
        if (ds2>0) {
            dAbsolute2 = dist(O_old.x,O_old.y,xLoop,yLoop)/abs(ds2);
        } else {
            dAbsolute2 = 0.0;
        }
        theta_1 = theta_1-sign(df)*atan((3.0*pow(dTot,2.0) * CfT1[0] + 2.0*dTot * CfT1[1] + CfT1[2])*dAbsolute1);
        theta_2 = theta_2-sign(df)*atan((3.0*pow(dTot,2.0) * CfT2[0] + 2.0*dTot * CfT2[1] + CfT2[2])*dAbsolute2);
    }

    addBorderElement(zFixed,zBorder,tube,p,dfStart,linesB,index,O,theta_2); 

    lineTemp = createLineTemp(xLoop,O.x,yLoop,O.y,theta_1, theta_2); 
    lineTemp.ind = index;

    if (dfStart<0.0) {
        lines.line.insert(lines.line.begin(), lineTemp);
    } else {
        lines.line.push_back(lineTemp);
    }
    }
}

void vectorFieldMap::newTubeSplit(hip_msgs::tubes tube,double zFixed,double zBorder,double zVar,double zVarEnd,double pVar,double df,double v1[2],hip_msgs::lines &lines,hip_msgs::lines &linesB) {
lines.line.clear();
linesB.line.clear();
double ATemp[3][3] = {{0.0, 0.0, 1.0}, {3.0*pow(fabs(df),2.0), 2.0*fabs(df), 1.0}, {pow(fabs(df),3.0), pow(fabs(df),2.0), fabs(df)}};

mat::fixed<3,3> A;
for (int i=0;i<3;i++) {
    for (int j=0;j<3;j++) {
        A[j*3+i] = ATemp[i][j];
    }
}
vec b = {{0.0}, {0.0}, {zVarEnd-zVar}};

vec c = pinv(A) * b;
vector<double> CfT1, CfT2;
CfT1.push_back(c[0]);
CfT1.push_back(c[1]);
CfT1.push_back(c[2]);

CfT2.push_back(0.0);
CfT2.push_back(0.0);
CfT2.push_back(0.0);

vectorFieldMap::positionVars O;

bool endReached = true;
double dfStart = df;
int index;
index = findPGlobalInTube(tube,pVar);
pVar = (pVar-tube.tube[index].main.pC)/tube.tube[index].main.pM;
O = Fry(tube.tube[index].main,pVar,zVar,1);
double xLoop = O.x;
double yLoop = O.y;
double dTot = 0.0;
O = Fry(tube.tube[index].main,pVar,zFixed,1);

hip_msgs::line lineTemp;
lineTemp = createLineTemp(xLoop,O.x,yLoop,O.y,0.0,0.0);
double theta_2 = relativeGradientV2(lineTemp,tube.tube[index].main,2);
lineTemp = createLineTemp(xLoop,O.x,yLoop,O.y,angleBetweenVectors(lineTemp,v1),theta_2);
lineTemp.ind = index;

lines.line.push_back(lineTemp);
linesB.line.clear();
addBorderElement(zFixed,zBorder,tube,pVar,dfStart,linesB,index,O,theta_2);
bool first = true;

while (endReached) {
    // [df,dTot,xLoop,yLoop,endReached,lines,linesB] = 
    splitFrontBack(xLoop,yLoop,df,tube,endReached,lines,linesB,dTot,zVar,CfT1,CfT2,zFixed,zBorder,dfStart);
    first = false;
}
endReached = false;
CfT1[0] = 0.0;
CfT1[1] = 0.0;
CfT1[2] = 0.0;
double d = sign(dfStart)*1000.0;
splitFrontBack(xLoop,yLoop,d,tube,endReached,lines,linesB,dTot,zVarEnd,CfT1,CfT2,zFixed,zBorder,dfStart);
}

void vectorFieldMap::walkStraight(double x1,double y1,double x2,double y2,hip_msgs::tubes tube,vector<int> tube_indices, vector<double> x_ind, vector<double> y_ind) {
    tube_indices.clear();
    x_ind.clear();
    y_ind.clear();
    int i;

    double dx, dy;

    dx = x2-x1;
    dy = y2-y1;
    vector<int> index_oldTemp,index;
    double index_old;
    findPointInTube(tube,x1,y1,index_oldTemp);
    if (index_oldTemp.size()>0) {
    index_old = index_oldTemp[0];
    double x,y;

    for (int i = 1; i<100; i++) {
        x = x1+double(i)*dx/100.0;
        y = y1+double(i)*dy/100.0;
        findPointInTube(tube,x,y,index);
        if (index.size()==1) {
            if (index[0]!=index_old) {
                tube_indices.push_back(index[0]);
                x_ind.push_back(x);
                y_ind.push_back(y);
                index_old=index[0];
            }
        }
    }
    }
}

void vectorFieldMap::newTubeSplitMiddle(hip_msgs::tubes tube,double zFixed,double x[4],double y[4],int ind1,int ind2,double v1[2],double zBorder,hip_msgs::lines &lines,hip_msgs::lines &linesB) {
    lines.line.clear();
    linesB.line.clear();
    bool DUMMYBOOL;
    vector<int> tube_indicesTemp;
    vector<double> x_indTemp,y_indTemp;
    double zTemp, pTemp, theta_1, theta_2;
    struct positionVars O; 
    hip_msgs::line lineTemp;
    walkStraight(x[ind1],y[ind1],x[ind2],y[ind2],tube,tube_indicesTemp, x_indTemp, y_indTemp);
    for (int i = tube_indicesTemp.size()-1;i>-1;i = i - 1) { 
        zVal(tube.tube[tube_indicesTemp[i]].main,x_indTemp[i],y_indTemp[i],1,zTemp,pTemp,DUMMYBOOL);
        O = Fry(tube.tube[tube_indicesTemp[i]].main,pTemp,zFixed,1);
        lineTemp = createLineTemp(x_indTemp[i],O.x,y_indTemp[i],O.y,0.0,0.0);
        theta_2 = relativeGradientV2(lineTemp,tube.tube[tube_indicesTemp[i]].main,2);
        theta_1 = angleBetweenVectors(lineTemp,v1);
        lineTemp = createLineTemp(x_indTemp[i],O.x,y_indTemp[i],O.y,theta_1,theta_2);

        lineTemp.ind = tube_indicesTemp[i];
        lines.line.push_back(lineTemp);

        addBorderElement(zFixed,zBorder,tube,pTemp,-1,linesB,tube_indicesTemp[i],O,theta_2);

    }
}

void vectorFieldMap::addObject(hip_msgs::tubes &tube,hip_msgs::robot object, int TBCi) {
struct TBCS TBC;
double DUMMYDOUBLE,DUMMYDOUBLE2;
bool DUMMYBOOL;
TBC.i = TBCi;
TBC.border = false;
// obtain basic geometric information of robot in global tube
double theta, l, b, ct, st;
theta = object.theta;
l = object.l;
b = object.b;
ct = cos(theta);
st = sin(theta);

double x[4];
double y[4];

x[0] = -l/2.0*ct+b/2.0*st+object.x;
x[1] = -l/2.0*ct-b/2.0*st+object.x;
x[2] =  l/2.0*ct-b/2.0*st+object.x;
x[3] =  l/2.0*ct+b/2.0*st+object.x;

y[0] = -l/2.0*st-b/2.0*ct+object.y;
y[1] = -l/2.0*st+b/2.0*ct+object.y;
y[2] =  l/2.0*st+b/2.0*ct+object.y;
y[3] =  l/2.0*st-b/2.0*ct+object.y;

int count = 0;
vector<int> in, out, ind;
int indTot[4];
vectorFieldMap::positionVars O, O1, O2;
double v1[2];

for (int i = 0; i<4;i++) {
     findPointInTube(tube,x[i],y[i],ind);
    if (ind.size()>0) {
        indTot[i] = ind[0];
    } else {
        indTot[i] = -1;
    }
    if (indTot[i]!=-1) {
        count = count + 1;
        in.push_back(i);
    } else {
        out.push_back(i);
    }
}
if (count>0) {
    int index = -1;
    double zN, p, z, pN;
    bool inside;
    if (count<4) {
        for (int i = 0; i<out.size(); i++) {
            for (int j = 0; j<tube.tube.size(); j++) {
                zVal(tube.tube[j].main,x[out[i]],y[out[i]],1,z,p,inside);
                if (j == 0 && tube.tube[j].main.pM*p+tube.tube[j].main.pC<0.0) {
                    p = (0.01-tube.tube[j].main.pC)/tube.tube[j].main.pM;
                } else if (j == tube.tube.size()-1 && tube.tube[j].main.pM*p+tube.tube[j].main.pC>1.0) {
                    p = (0.99-tube.tube[j].main.pC)/tube.tube[j].main.pM;
                }
                
                if (findPGlobalInTube(tube,tube.tube[j].main.pM*p+tube.tube[j].main.pC)==j) {
                    index = j;
                    pN = p;
                    zN = z;
                }
            }
                

            if (index!=-1) {
                if (zN<0.0) {
                    zN = 0.001;
                } else if (z>1.0) {
                    zN = 0.999;
                }
                O = Fry(tube.tube[indTot[in[0]]].main,pN,zN,1);
                x[out[i]] = O.x;
                y[out[i]] = O.y;
                indTot[out[i]] = indTot[in[0]];
                in.push_back(out[i]);
            }
        }  
    }

    double zMat[4];
    double pMat[4];
    double distList[4];
    double pr[4];
    double dista;
    int i;
    int indT;
    int indBox[4];
    double xTemp,yTemp;

    for (int j = 0; j<in.size(); j++) {
        i = in[j];
        indT = indTot[i];
        // ind = ind[0];
        zVal(tube.tube[indT].main,x[i],y[i],1,z,p,inside);
        zMat[i] = z;
        pMat[i] = p;
        xTemp = x[i];
        yTemp = y[i];
        walkToBorder(xTemp,yTemp,-1000.0,tube,DUMMYBOOL,dista,DUMMYBOOL);
        distList[i] = dista;
        if (tube.tube[indT].main.curved) {
            pr[i] = pMat[i]*tube.tube[indT].main.pM + tube.tube[indT].main.pC;
        } else {
            pr[i] = pMat[i]*tube.tube[indT].main.pM + tube.tube[indT].main.pC;
        }
        indBox[i] = indT;
    }
    vector<double> prTemp;
    for (int i = 0; i<4;i++) {
        prTemp.push_back(pr[i]);
    }
    double val[4];

    for (int i = 0; i<4; i++) {
        minVal(prTemp,val[i],ind[i]);
        prTemp[ind[i]] = 1000000000.0;
    }

    int top[4], bottom[4];

    if (zMat[ind[0]]>zMat[ind[1]]) {
        top[0] = ind[0];
        bottom[0] = ind[1];
    } else {
        top[0] = ind[1];
        bottom[0] = ind[0];
    }
    if (zMat[ind[2]]>zMat[ind[3]]) {
        top[1] = ind[2];
        bottom[1] = ind[3];
    } else {
        top[1] = ind[3];
        bottom[1] = ind[2];
    }




    double dsMax = object.dsMax;
    double zMax, zMin;
    if (zMat[top[0]]>zMat[top[1]]) {
        xTemp = x[top[0]];
        yTemp = y[top[0]];
        walkConstant(tube.tube[indBox[top[0]]].main,dsMax,xTemp,yTemp,zMax,DUMMYDOUBLE,DUMMYDOUBLE2);

    } else {
        xTemp = x[top[1]];
        yTemp = y[top[1]];
        walkConstant(tube.tube[indBox[top[1]]].main,dsMax,xTemp,yTemp,zMax,DUMMYDOUBLE,DUMMYDOUBLE2);

    }

    if (zMat[bottom[0]]<zMat[bottom[1]]) {
        xTemp = x[bottom[0]];
        yTemp = y[bottom[0]];
        walkConstant(tube.tube[indBox[bottom[0]]].main,-dsMax,xTemp,yTemp,zMin,DUMMYDOUBLE,DUMMYDOUBLE2);

    } else {
        xTemp = x[bottom[1]];
        yTemp = y[bottom[1]];
        walkConstant(tube.tube[indBox[bottom[1]]].main,-dsMax,xTemp,yTemp,zMin,DUMMYDOUBLE,DUMMYDOUBLE2);

    }

    // create subtube elements
    hip_msgs::lines lines;
    hip_msgs::lines linesB;

    double zVarEnd, zBorder, zFixed;
    int ind1, ind2;
    vector<double> zTemp;
    double outterZ;
    int outterZind;

    for (int i = 0; i<4; i++) {
        zTemp.push_back(zMat[i]);
    }

    // "manual" variables
    if (TBC.i==0 || TBC.i==1) {
        if (TBC.i == 0) {
            zVarEnd = 0.0;
            zBorder = 1.0;
            zFixed = zMax;
            ind1 = top[0];
            ind2 = top[1];
            maxVal(zTemp,outterZ, outterZind);
        } else if (TBC.i==1) {
            zVarEnd = 1.0;
            zBorder = 0.0;
            zFixed = zMin;
            ind1 = bottom[0];
            ind2 = bottom[1];
            minVal(zTemp, outterZ, outterZind);
        }

        
        O1 = Fry(tube.tube[indBox[outterZind]].main,pMat[outterZind],outterZ,1);
        O2 = Fry(tube.tube[indBox[outterZind]].main,pMat[outterZind],zBorder,1);

        double d = dist(O1.x,O1.y,O2.x,O2.y);
        double zVar1, pVar1, zVar2, pVar2;
        if (d>0.4) {

            v1[0] = x[ind2]-x[ind1];
            v1[1] = y[ind2]-y[ind1];
            
            // automatic generated variables
            zVar1 = zMat[ind1];
            pVar1 = pr[ind1];
            zVar2 = zMat[ind2];
            pVar2 = pr[ind2];
            // plotLine(x[top[0]], x[bottom[0]], y[top[0]], y[bottom[0]],dynamicMap.markers.size() ,1, 0, 0.5, 0.9, "robotF",dynamicMap);

            O1 = Fry(tube.tube[indBox[ind1]].main,pMat[ind1],zVar1,1);
            O2 = Fry(tube.tube[indBox[ind1]].main,pMat[ind1],zVarEnd,1);
            
            dista = dist(O2.x, O2.y, O1.x, O1.y);            

            // hip_msgs::line linesTemp, linesTempB;
            hip_msgs::lines linesTemp, linesTempB;

            newTubeSplit(tube,zFixed,zBorder,zVar1,zVarEnd,pVar1,-dista*object.df,v1,linesTemp,linesTempB);


            for (int i = 0; i<linesTemp.line.size();i++) {
                lines.line.push_back(linesTemp.line[i]);
            }
            for (int i = 0; i<linesTempB.line.size();i++) {
                linesB.line.push_back(linesTempB.line[i]);
            }
            // linesB.line.push_back(linesTempB);

            newTubeSplitMiddle(tube,zFixed,x,y,ind1,ind2,v1,zBorder,linesTemp,linesTempB);
            for (int i = 0; i<linesTemp.line.size();i++) {
                lines.line.push_back(linesTemp.line[i]);
            }
            for (int i = 0; i<linesTempB.line.size();i++) {
                linesB.line.push_back(linesTempB.line[i]);
            }

            O1 = Fry(tube.tube[indBox[ind1]].main,pMat[ind2],zVar2,1);
            O2 = Fry(tube.tube[indBox[ind1]].main,pMat[ind2],zVarEnd,1);
            
            dista = dist(O2.x,O2.y,O1.x,O1.y);
            
            newTubeSplit(tube,zFixed,zBorder,zVar2,zVarEnd,pVar2,dista*object.db,v1,linesTemp,linesTempB);
            for (int i = 0; i<linesTemp.line.size();i++) {
                lines.line.push_back(linesTemp.line[i]);
            }
            for (int i = 0; i<linesTempB.line.size();i++) {
                linesB.line.push_back(linesTempB.line[i]);
            }
            
            if (linesB.line.size()>0) {
                TBC.oneSide = true;
                TBC.border = true;
                linesToTube(linesB,tube,0,TBC,1);
                TBC.border = false;
            } else {
                TBC.oneSide = false;
            } 
            linesToTube(lines,tube,0,TBC,1);
        }
    } else if (TBC.i==2) {

        int frontT, frontB, backT, backB;
        TBC.oneSide = true;
        frontT  = top[0];
        frontB = bottom[0];
        backT = top[1];
        backB = bottom[1];

        v1[0] = x[frontT]-x[backT];
        v1[1] = y[frontT]-y[backT];

        double xBorderB,yBorderB, zBorderB, pBorderB;
        xBorderB = (x[backT]+x[backB])/2.0;
        yBorderB = (y[backT]+y[backB])/2.0;

        vector<int> indexTemp;
        findPointInTube(tube,xBorderB,yBorderB,indexTemp);
        if (indexTemp.size()>0) {
            index = indexTemp[0];
            
            zVal(tube.tube[index].main,xBorderB,yBorderB,1,zBorderB,pBorderB,DUMMYBOOL);
            
            O = Fry(tube.tube[index].main,pBorderB,0,1);
            dista = dist(O.x, O.y, xBorderB, yBorderB);
            double distTube = dist(x[frontT], y[frontT], x[backT], y[backT]);
            
            hip_msgs::lines linesTemp, linesTempDummy;

            newTubeSplit(tube,zBorderB,-1.0,zMat[backB],0,pr[backB],-dista*object.df-distTube,v1,linesTemp,linesTempDummy); 

            // lines.line.push_back(linesTemp);
            for (int i = 0; i<linesTemp.line.size();i++) {
                lines.line.push_back(linesTemp.line[i]);
            }

            TBC.border = false;
            linesToTube(lines,tube,0,TBC,0);

            O = Fry(tube.tube[index].main,pBorderB,1,1);
            dista = dist(O.x, O.y, xBorderB, yBorderB);
    
            newTubeSplit(tube,zBorderB,-1.0,zMat[backT],1,pr[backT],-dista*object.df-distTube,v1,linesTemp,linesTempDummy);

            // lines.line.push_back(linesTemp);
            for (int i = 0; i<linesTemp.line.size();i++) {
                lines.line.push_back(linesTemp.line[i]);
            }
                
            TBC.border = true;
            linesToTube(lines,tube,0,TBC,1);
        }
    }
}
}

void vectorFieldMap::plotRobot(hip_msgs::robot object, string robotFrame, double markerLifetime) {
    double theta, l, b, ct, st;
    theta = object.theta;
    l = object.l;
    b = object.b;
    ct = cos(theta);
    st = sin(theta);
    double x[4], y[4];

    x[0] = -l/2.0*ct+b/2.0*st+object.x;
    x[1] = -l/2.0*ct-b/2.0*st+object.x;
    x[2] =  l/2.0*ct-b/2.0*st+object.x;
    x[3] =  l/2.0*ct+b/2.0*st+object.x;

    y[0] = -l/2.0*st-b/2.0*ct+object.y;
    y[1] = -l/2.0*st+b/2.0*ct+object.y;
    y[2] =  l/2.0*st+b/2.0*ct+object.y;
    y[3] =  l/2.0*st-b/2.0*ct+object.y;

//    string frame = robotFrame;
    plotLine(x[0], x[1], y[0], y[1],dynamicMap.markers.size() ,0, 1, 0.5, 0.7, "robot", robotFrame, dynamicMap, markerLifetime);
    plotLine(x[1], x[2], y[1], y[2],dynamicMap.markers.size() ,0, 1, 0.5, 0.7, "robot", robotFrame, dynamicMap, markerLifetime);
    plotLine(x[2], x[3], y[2], y[3],dynamicMap.markers.size() ,0, 1, 0.5, 0.7, "robot", robotFrame, dynamicMap, markerLifetime);
    plotLine(x[3], x[0], y[3], y[0],dynamicMap.markers.size() ,0, 1, 0.5, 0.7, "robot", robotFrame, dynamicMap, markerLifetime);

}

void vectorFieldMap::initializeMap() {
    cout<<"initialize map..."<< endl;
    TBCS TBC;
    hip_msgs::lines lines;
    hip_msgs::line lineTemp;

    robot.x = 6.5;
    robot.y = 8.5;
    robot.l = 1.2;
    robot.b = 0.75;
    robot.theta = M_PI;
    robot.df = 2.0;
    robot.db = 2.0;
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
//
    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(0.0,2.5,0.0,0.0,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);
    
    lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,0.0,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(0.0,2.5,-0.1,-0.1,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // linesToTube(lines,globalTube,0,TBC,1);
    
    lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(2.2,2.2,0,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    
    // lineTemp = createLineTemp(0.0,0.0,0,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // linesToTube(lines,globalTube,0,TBC,0);
    // lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    lines.line.push_back(lineTemp);

    lineTemp = createLineTemp(0.0,2.5,0.0,0.0,0.0,0.0);
    lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,0);
    lines.line.push_back(lineTemp);

    
    lineTemp = createLineTemp(0.0,2.5,-0.1,-0.1,0.0,0.0);
    lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    linesToTube(lines,globalTube,0,TBC,1);
    // lines.line.push_back(lineTemp);
//
    lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
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
    
    // lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(2.5,2.5,0.0,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // linesToTube(lines,globalTube,0,TBC,0);
    
    // lineTemp = createLineTemp(0.0,2.5,2.5,2.5,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // lineTemp = createLineTemp(0.0,2.3,0.0,0.0,0.0,0.0);
    // lines.line.push_back(lineTemp);

    // linesToTube(lines,globalTube,0,TBC,0);
    
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

    double markerLifetime = -1;
    for (int i=0;i<globalTube.tube.size();i++) {
        plotTube(globalTube.tube[i].main,20,5,staticMap.markers.size(),0,0,1.0,1.0,1.0,0.8,"staticMapTubes",staticMap, markerLifetime);
    }
    createGraph(globalTube, G);
    createGraphShort(globalTube, G);
    cout<<"Static map created"<< endl;
}