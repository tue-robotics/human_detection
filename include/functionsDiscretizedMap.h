#ifndef HIP_FUNCTIONS_DISCRETIZED_MAP_H_
#define HIP_FUNCTIONS_DISCRETIZED_MAP_H_

#include <math.h>
#include <vector>
#include <iostream>
#include <armadillo>

#include <hip_msgs/line.h>
#include <hip_msgs/lines.h>
#include <hip_msgs/tube.h>
#include <hip_msgs/singleTube.h>
#include <hip_msgs/tubes.h>
#include <hip_msgs/tubesH.h>
#include <hip_msgs/transform.h>
#include <hip_msgs/human.h>
#include <hip_msgs/robot.h>
#include <hip_msgs/hypothesis.h>
#include <hip_msgs/hypotheses.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace arma;

class vectorFieldMap {
    public:
    void initializeMap();
    void updateHypotheses(double x, double y, double vx, double vy, double xRobot, double yRobot, double thetaRobot, string robotFrame, string mapFrame);
    void readMap(visualization_msgs::MarkerArray &staticMarkers,visualization_msgs::MarkerArray &dynamicMarkers);
    hip_msgs::tubes globalTube;
    hip_msgs::tubesH globalTubesH;
    hip_msgs::hypotheses hypotheses;
    // vector<hip_msgs::tubes> tubesH;

    struct positionVars {
        float x;
        float y;
        bool curved;
    };

    struct graph {
        vector<vector<double>> A;
        vector<vector<double>> AShort;
        vector<bool> marked;
        vector<int> ALink;
    };

    struct recursiveWalk {
        int toTube;
        double dir;
        double distance;
        int fromTube;
        double pEndStore;
        double zEndStore;
        vector<int> storeElement;
    };

    struct recursiveWalkStore {
        int index;
        double dTot;
        double pStore;
        vector<int> path;
    };

    private:
    hip_msgs::human human;
    hip_msgs::robot robot;
    visualization_msgs::MarkerArray staticMap;
    visualization_msgs::MarkerArray dynamicMap;
    visualization_msgs::MarkerArray map;
    graph G;

    struct TBCS {
        int i;
        bool border;
        bool oneSide;
    };

    void pointsToLine(hip_msgs::lines &lines);
    double dist(double x1,double y1,double x2,double y2);
    double sign(double var);
    void intersectLines(hip_msgs::lines lines, positionVars &result);
    void initializeTubeCurved(hip_msgs::singleTube &tube);
    void initializeTubeStraight(hip_msgs::singleTube &tube);
    void createGraphShort(hip_msgs::tubes tube, graph &G);
    void zVal(hip_msgs::singleTube &tube,double x,double y,bool rel,double &z, double &p, bool &inside);
    hip_msgs::line createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2);
    void linspace(double v1, double v2, int n, vector<double> &result);
    positionVars Fry(hip_msgs::singleTube tube,double I1,double I2,bool rel);
    void clearTube(visualization_msgs::MarkerArray map);
    void plotTube(hip_msgs::singleTube tube, int n1, int n2,int subID, bool rel, bool sub, double r, double g, double b, double a, string ns, visualization_msgs::MarkerArray &map);
    void linesToTube(hip_msgs::lines &lines,hip_msgs::tubes &tubes,double id,TBCS TBC,bool marked);
    void createGraph(hip_msgs::tubes tube, graph &G);
    void findPointInTube(hip_msgs::tubes tubes, double x, double y, vector<int> &index);
    void calcGradient(hip_msgs::singleTube tube,double x, double y, double &u, double &v);
    void derivativeF(hip_msgs::singleTube tube, double x, double y, double &dphidx, double &dphidy);
    void FuvStraight(double dphidx,double dphidy,hip_msgs::singleTube tube,double &u, double &v);
    void FuvCurved(double dphidr,double dphidtheta,double theta,double r,double signlr,double &u, double &v);
    bool considerHuman(hip_msgs::human person,hip_msgs::robot robot);
    void createDynamicAsOI(hip_msgs::tubes tube,double xPR, double yPR,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax,vector<recursiveWalkStore> &store);
    void distEnd(double x,double y,vector<recursiveWalkStore> &store,vector<recursiveWalk> prev,vector<recursiveWalk> &next,hip_msgs::tubes tube,graph G,vector<bool> openTubes,double dMax);
    void nextTube(vector<int> index,graph G,vector<int> &toTube, vector<double> &direction);
    bool inAvailableTube(vector<bool> openTubes,graph G,int toTube);
    void nextTube(int index,graph G,vector<int> &toTube, vector<double> &direction);
    void recursiveWalkA(vector<recursiveWalkStore> &store,vector<recursiveWalk> next,hip_msgs::tubes tube,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax);
    void validateHypotheses(double x, double y, double vx, double vy, vector<hip_msgs::tubes> tubesH);
    void createTubeHypothesis(hip_msgs::tubes tube,vector<recursiveWalkStore> store,graph G,hip_msgs::robot robot, vector<hip_msgs::tubes> &tubesH);
    void borderTransform(hip_msgs::tubes &tube);
    void plotAOI(hip_msgs::singleTube tube, int n, int subID, double a, string ns,double p,visualization_msgs::MarkerArray &map);
    void walkConstant(hip_msgs::singleTube tube,double ds,double &x,double &y, double &C, double &p, double &D);
    void addObject(hip_msgs::tubes &tube,hip_msgs::robot object, int TBCi);
    void walkToBorder(double &x,double &y,double dMax,hip_msgs::tubes tube,bool &endReached, double &dist, bool &border);
    void minVal(vector<double> prTemp, double &minVal, int &minInd);
    void maxVal(vector<double> prTemp, double &minVal, int &minInd);
    int findPGlobalInTube(hip_msgs::tubes tube, double p);
    double angleBetweenVectors(hip_msgs::line lineTemp,double v1[2]);
    double relativeGradientV2(hip_msgs::line lineTemp,hip_msgs::singleTube tube,int i);
    void addBorderElement(double zFixed,double zBorder,hip_msgs::tubes tube,double p,double dfStart,hip_msgs::lines linesB,double index,positionVars O,double theta_2);
    void splitFrontBack(double &xLoop,double &yLoop,double &df,hip_msgs::tubes tube,bool &endReached,hip_msgs::lines &lines,hip_msgs::lines &linesB,double &dTot,double zVar,vector<double> CfT1,vector<double> CfT2,double zFixed,double zBorder,double dfStart);
    void newTubeSplit(hip_msgs::tubes tube,double zFixed,double zBorder,double zVar,double zVarEnd,double pVar,double df,double v1[2],hip_msgs::lines &lines,hip_msgs::lines &linesB);
    void walkStraight(double x1,double y1,double x2,double y2,hip_msgs::tubes tube,vector<int> tube_indices, vector<double> x_ind, vector<double> y_ind);
    void newTubeSplitMiddle(hip_msgs::tubes tube,double zFixed,double x[4],double y[4],int ind1,int ind2,double v1[2],double zBorder,hip_msgs::lines &lines,hip_msgs::lines &linesB);
    void plotLine(double x1, double x2, double y1, double y2, int i,double r, double g, double b, double a, string ns, string frame, visualization_msgs::MarkerArray &map);
    void plotRobot(hip_msgs::robot object, string robotFrame);
    void validatePointInSubTube(hip_msgs::tube tube, double x, double y, double vx, double vy,vector<double> &prop);
    void plotStandingTube(double x, double y, double radius, double r, double g, double b, double a, string ns,visualization_msgs::MarkerArray &map, string mapFrame);
};

#endif // HIP_FUNCTIONS_DISCRETIZED_MAP_H_