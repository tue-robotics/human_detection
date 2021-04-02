#ifndef HIP_FUNCTIONS_DISCRETIZED_MAP_H_
#define HIP_FUNCTIONS_DISCRETIZED_MAP_H_

#include <math.h>
#include <vector>
#include <iostream>
#include <armadillo>

#include <human_intention_prediction/line.h>
#include <human_intention_prediction/lines.h>
#include <human_intention_prediction/tube.h>
#include <human_intention_prediction/singleTube.h>
#include <human_intention_prediction/tubes.h>
#include <human_intention_prediction/tubesH.h>
#include <human_intention_prediction/transform.h>
#include <human_intention_prediction/human.h>
#include <human_intention_prediction/robot.h>
#include <human_intention_prediction/hypothesis.h>
#include <human_intention_prediction/hypotheses.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace arma;

class vectorFieldMap {
    public:
    void initializeMap();
    void updateHypotheses(double x, double y, double vx, double vy, double xRobot, double yRobot, double thetaRobot, string robotFrame, string mapFrame);
    void readMap(visualization_msgs::MarkerArray &staticMarkers,visualization_msgs::MarkerArray &dynamicMarkers);
    human_intention_prediction::tubes globalTube;
    human_intention_prediction::tubesH globalTubesH;
    human_intention_prediction::hypotheses hypotheses;
    // vector<human_intention_prediction::tubes> tubesH;

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
    human_intention_prediction::human human;
    human_intention_prediction::robot robot;
    visualization_msgs::MarkerArray staticMap;
    visualization_msgs::MarkerArray dynamicMap;
    visualization_msgs::MarkerArray map;
    graph G;

    struct TBCS {
        int i;
        bool border;
        bool oneSide;
    };

    void pointsToLine(human_intention_prediction::lines &lines);
    double dist(double x1,double y1,double x2,double y2);
    double sign(double var);
    void intersectLines(human_intention_prediction::lines lines, positionVars &result);
    void initializeTubeCurved(human_intention_prediction::singleTube &tube);
    void initializeTubeStraight(human_intention_prediction::singleTube &tube);
    void createGraphShort(human_intention_prediction::tubes tube, graph &G);
    void zVal(human_intention_prediction::singleTube &tube,double x,double y,bool rel,double &z, double &p, bool &inside);
    human_intention_prediction::line createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2);
    void linspace(double v1, double v2, int n, vector<double> &result);
    positionVars Fry(human_intention_prediction::singleTube tube,double I1,double I2,bool rel);
    void clearTube(visualization_msgs::MarkerArray map);
    void plotTube(human_intention_prediction::singleTube tube, int n1, int n2,int subID, bool rel, bool sub, double r, double g, double b, double a, string ns, visualization_msgs::MarkerArray &map);
    void linesToTube(human_intention_prediction::lines &lines,human_intention_prediction::tubes &tubes,double id,TBCS TBC,bool marked);
    void createGraph(human_intention_prediction::tubes tube, graph &G);
    void findPointInTube(human_intention_prediction::tubes tubes, double x, double y, vector<int> &index);
    void calcGradient(human_intention_prediction::singleTube tube,double x, double y, double &u, double &v);
    void derivativeF(human_intention_prediction::singleTube tube, double x, double y, double &dphidx, double &dphidy);
    void FuvStraight(double dphidx,double dphidy,human_intention_prediction::singleTube tube,double &u, double &v);
    void FuvCurved(double dphidr,double dphidtheta,double theta,double r,double signlr,double &u, double &v);
    bool considerHuman(human_intention_prediction::human person,human_intention_prediction::robot robot);
    void createDynamicAsOI(human_intention_prediction::tubes tube,double xPR, double yPR,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax,vector<recursiveWalkStore> &store);
    void distEnd(double x,double y,vector<recursiveWalkStore> &store,vector<recursiveWalk> prev,vector<recursiveWalk> &next,human_intention_prediction::tubes tube,graph G,vector<bool> openTubes,double dMax);
    void nextTube(vector<int> index,graph G,vector<int> &toTube, vector<double> &direction);
    bool inAvailableTube(vector<bool> openTubes,graph G,int toTube);
    void nextTube(int index,graph G,vector<int> &toTube, vector<double> &direction);
    void recursiveWalkA(vector<recursiveWalkStore> &store,vector<recursiveWalk> next,human_intention_prediction::tubes tube,graph G,vector<bool> &seenTubes,vector<bool> openTubes,double dMax);
    void validateHypotheses(double x, double y, double vx, double vy, vector<human_intention_prediction::tubes> tubesH);
    void createTubeHypothesis(human_intention_prediction::tubes tube,vector<recursiveWalkStore> store,graph G,human_intention_prediction::robot robot, vector<human_intention_prediction::tubes> &tubesH);
    void borderTransform(human_intention_prediction::tubes &tube);
    void plotAOI(human_intention_prediction::singleTube tube, int n, int subID, double a, string ns,double p,visualization_msgs::MarkerArray &map);
    void walkConstant(human_intention_prediction::singleTube tube,double ds,double &x,double &y, double &C, double &p, double &D);
    void addObject(human_intention_prediction::tubes &tube,human_intention_prediction::robot object, int TBCi);
    void walkToBorder(double &x,double &y,double dMax,human_intention_prediction::tubes tube,bool &endReached, double &dist, bool &border);
    void minVal(vector<double> prTemp, double &minVal, int &minInd);
    void maxVal(vector<double> prTemp, double &minVal, int &minInd);
    int findPGlobalInTube(human_intention_prediction::tubes tube, double p);
    double angleBetweenVectors(human_intention_prediction::line lineTemp,double v1[2]);
    double relativeGradientV2(human_intention_prediction::line lineTemp,human_intention_prediction::singleTube tube,int i);
    void addBorderElement(double zFixed,double zBorder,human_intention_prediction::tubes tube,double p,double dfStart,human_intention_prediction::lines linesB,double index,positionVars O,double theta_2);
    void splitFrontBack(double &xLoop,double &yLoop,double &df,human_intention_prediction::tubes tube,bool &endReached,human_intention_prediction::lines &lines,human_intention_prediction::lines &linesB,double &dTot,double zVar,vector<double> CfT1,vector<double> CfT2,double zFixed,double zBorder,double dfStart);
    void newTubeSplit(human_intention_prediction::tubes tube,double zFixed,double zBorder,double zVar,double zVarEnd,double pVar,double df,double v1[2],human_intention_prediction::lines &lines,human_intention_prediction::lines &linesB);
    void walkStraight(double x1,double y1,double x2,double y2,human_intention_prediction::tubes tube,vector<int> tube_indices, vector<double> x_ind, vector<double> y_ind);
    void newTubeSplitMiddle(human_intention_prediction::tubes tube,double zFixed,double x[4],double y[4],int ind1,int ind2,double v1[2],double zBorder,human_intention_prediction::lines &lines,human_intention_prediction::lines &linesB);
    void plotLine(double x1, double x2, double y1, double y2, int i,double r, double g, double b, double a, string ns, string frame, visualization_msgs::MarkerArray &map);
    void plotRobot(human_intention_prediction::robot object, string robotFrame);
    void validatePointInSubTube(human_intention_prediction::tube tube, double x, double y, double vx, double vy,vector<double> &prop);
    void plotStandingTube(double x, double y, double radius, double r, double g, double b, double a, string ns,visualization_msgs::MarkerArray &map, string mapFrame);
};

#endif // HIP_FUNCTIONS_DISCRETIZED_MAP_H_