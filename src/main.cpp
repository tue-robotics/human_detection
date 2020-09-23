#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <human_walking_detection/walls.h>
#include <human_walking_detection/wall.h>
#include <human_walking_detection/Pose.h>
#include <ed_gui_server/objsPosVel.h>
#include <functions.h>
#include <rosnode.h>
#include <functionsDiscretizedMap.h>
#include <human_walking_detection/hypothesis.h>
#include <human_walking_detection/hypotheses.h>

using namespace std;



/// [main]
int main(int argc, char** argv)
/// [main]
{

    ros::init(argc, argv, "HWD");
    rosNode rosNode;
    vectorFieldMap map;
    visualization_msgs::MarkerArray staticMarkers;
    visualization_msgs::MarkerArray dynamicMarkers;
    rosNode.initialize();
    map.initializeMap();
    map.readMap(staticMarkers,dynamicMarkers);
    rosNode.setStaticMap(staticMarkers);

    /// [loop start]
    int i=0;
    double likelihood;
    double v_x = 1.0;
    double v_y = 0.3;
    // string text;
    
    matrix A,H,P,Q,R,I;

    initializeKalman(A,H,P,Q,R,I,1.0/15.0);
    double prevIt = ros::Time::now().toSec();
    double dt;
    ros::Rate r(15); // loop at 15 hz

    double u,v,dist1,dist2,totP;

    while(ros::ok())//(i<rosNode.iMax)
    {
        i++;


        // if (i%500==0) {

        
        rosNode.publishTube(map.globalTube);
        map.updateHypotheses(rosNode.humanPosVel.x,rosNode.humanPosVel.y,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy);
        rosNode.publishHypotheses(map.hypotheses);
        map.readMap(staticMarkers,dynamicMarkers);
        // rosNode.removeDynamicMap();
        if (i%3==0) {
            rosNode.removeDynamicMap();
        }
        rosNode.setDynamicMap(dynamicMarkers);
        rosNode.publishMap();
        // }
        rosNode.visualizeHuman();
        rosNode.visualizeMeasuredHuman();
        rosNode.visualizeRobot();
        // cout<<mapMarkers.markers[0].points[0].x<<" "<<mapMarkers.markers[0].points[0].x<<endl;
        

    // check current vector field
        // if (movedEnough) {}
        //     updateHypothesis();
        // }
        // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0)); // Call ROS stream and wait 1000 sec if no new measurement

        dt = ros::Time::now().toSec()-prevIt;
        updateKalman(A,H,P,Q,R,I,rosNode.humanPosVel,rosNode.measurement,dt);
        cout<<"dt = "<<dt<<endl;
        prevIt = ros::Time::now().toSec();


        rosNode.publishHumanPV();

        if (rosNode.humanPosVel.x<2.5) {
            getLikelihood(1,0,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,-rosNode.humanPosVel.y,2.5-rosNode.humanPosVel.y,likelihood);
        } else if (rosNode.humanPosVel.x>4.8) {
            getLikelihood(-1,0,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,rosNode.humanPosVel.y-2.5,rosNode.humanPosVel.y,likelihood);
        } else {
            if (rosNode.humanPosVel.vx>0) {
                curvedVectorField1(rosNode.humanPosVel.x, rosNode.humanPosVel.y, u,v,dist1,dist2);
                cout<<"Up from left!"<<endl;
                getLikelihood(u,v,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,dist1,dist2,likelihood);
            } else {
                curvedVectorField2(rosNode.humanPosVel.x, rosNode.humanPosVel.y, u,v,dist1,dist2);
                cout<<"Up from right!"<<endl;
                getLikelihood(u,v,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,dist1,dist2,likelihood);
            }
        }
        cout<< "likelihood up is equal to: " << likelihood<<endl;
        rosNode.p2 = likelihood;
        getLikelihood(1,0,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,-rosNode.humanPosVel.y,2.5-rosNode.humanPosVel.y,likelihood);
        cout<< "likelihood right is equal to: " << likelihood<<endl;
        rosNode.p3 = likelihood;
        getLikelihood(-1,0,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,rosNode.humanPosVel.y-2.5,rosNode.humanPosVel.y,likelihood);
        cout<< "likelihood left is equal to: " << likelihood<<endl;
        rosNode.p1 = likelihood;
        rosNode.p4 = 0.2;

        totP = rosNode.p1 + rosNode.p2 + rosNode.p3 + rosNode.p4;
        rosNode.p1 = rosNode.p1/totP;
        rosNode.p2 = rosNode.p2/totP;
        rosNode.p3 = rosNode.p3/totP;
        rosNode.p4 = rosNode.p4/totP;
        rosNode.publishAoI();


        r.sleep();



    }
    /// [loop end]

    return 0;
}