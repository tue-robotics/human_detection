#include <math.h>
using namespace std;
#include <vector>
#include <human_walking_detection/line.h>
#include <human_walking_detection/lines.h>
#include <human_walking_detection/tube.h>
#include <human_walking_detection/singleTube.h>
#include <human_walking_detection/tubes.h>
#include <human_walking_detection/transform.h>
#include <human_walking_detection/human.h>
#include <human_walking_detection/robot.h>
#include <iostream>
#include <armadillo>
#include <visualization_msgs/MarkerArray.h>

using namespace arma;







class vectorFieldMap {
    public:
    void initializeMap();
    void readMap(visualization_msgs::MarkerArray &markers);
    human_walking_detection::tubes globalTube;

    struct positionVars {
        float x;
        float y;
        bool curved;
    };



    private:
    visualization_msgs::MarkerArray staticMap;
    

    struct TBCS {
        int i;
        bool border;
        bool oneSide;
    };


    void pointsToLine(human_walking_detection::lines &lines);
    double dist(double x1,double y1,double x2,double y2);
    double sign(double var);
    void intersectLines(human_walking_detection::lines lines, positionVars &result);
    void initializeTubeCurved(human_walking_detection::singleTube &tube);
    void initializeTubeStraight(human_walking_detection::singleTube &tube);
    void zVal(human_walking_detection::singleTube &tube,double x,double y,bool rel,double &z, double &p, bool &inside);
    human_walking_detection::line createLineTemp(double x1,double x2,double y1,double y2,double th1, double th2);
    void linspace(double v1, double v2, int n, vector<double> &result);
    positionVars Fry(human_walking_detection::singleTube tube,double I1,double I2,bool rel);
    void clearTube(visualization_msgs::MarkerArray map);
    void plotTube(human_walking_detection::singleTube tube, int n1, int n2,int subID, bool rel, bool sub, visualization_msgs::MarkerArray &map);
    void linesToTube(human_walking_detection::lines &lines,human_walking_detection::tubes &tubes,double id,TBCS TBC,bool marked);
};