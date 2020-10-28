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

    while(ros::ok())
    {
        i++;
        rosNode.publishTube(map.globalTube,map.globalTubesH);
        map.updateHypotheses(rosNode.humanPosVel.x,rosNode.humanPosVel.y,rosNode.humanPosVel.vx,rosNode.humanPosVel.vy,rosNode.xRobot,rosNode.yRobot,rosNode.thetaRobot);
        rosNode.publishHypotheses(map.hypotheses);
        map.readMap(staticMarkers,dynamicMarkers);
        if (i%3==0) {
            rosNode.removeDynamicMap();
        }
        rosNode.setDynamicMap(dynamicMarkers);
        rosNode.publishMap();
        // }
        rosNode.visualizeHuman();
        rosNode.visualizeMeasuredHuman();
        rosNode.visualizeRobot();
        
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
        // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0)); // Call ROS stream and wait 1 sec if no new measurement

        dt = ros::Time::now().toSec()-prevIt;
        updateKalman(A,H,P,Q,R,I,rosNode.humanPosVel,rosNode.measurement,dt);
        cout<<"dt = "<<dt<<endl;
        prevIt = ros::Time::now().toSec();
        rosNode.publishHumanPV();
        r.sleep();
    }
    /// [loop end]
    return 0;
}