#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <human_intention_prediction/walls.h>
#include <human_intention_prediction/wall.h>
#include <human_intention_prediction/Pose.h>
#include <ed_gui_server/objsPosVel.h>
#include <functions.h>
#include <rosnode.h>
#include <functionsDiscretizedMap.h>
#include <human_intention_prediction/hypothesis.h>
#include <human_intention_prediction/hypotheses.h>

using namespace std;

/// [main]
int main(int argc, char** argv)
{

    ros::init(argc, argv, "HIP");
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
    
    matrix A,H,P,Q,R,I; // Kalman filter matrices

    initializeKalman(A,H,P,Q,R,I,1.0/15.0);
    double prevIt = ros::Time::now().toSec();
    double dt;
    ros::Rate r(15); // loop at 15 hz

    double u,v,dist1,dist2,totP;

    ros::spinOnce();
    while(ros::ok())
    {
        i++;
        rosNode.publishTube(map.globalTube,map.globalTubesH);

        // Get yaw robot
        tf2::Quaternion q ( rosNode.robotPose.pose.pose.orientation.x, rosNode.robotPose.pose.pose.orientation.y, 
                            rosNode.robotPose.pose.pose.orientation.z, rosNode.robotPose.pose.pose.orientation.w );
        
        tf2::Matrix3x3 matrix ( q );
        double rollRobot, pitchRobot, yawRobot;
        matrix.getRPY ( rollRobot, pitchRobot, yawRobot );

        bool poseValid = rosNode.robotPose.pose.pose.position.x == rosNode.robotPose.pose.pose.position.x;
        poseValid = (poseValid && rosNode.robotPose.pose.pose.position.y == rosNode.robotPose.pose.pose.position.y);
        poseValid = (poseValid && yawRobot == yawRobot);

        if(poseValid)
        {
            map.updateHypotheses(rosNode.humanPosVel.x, rosNode.humanPosVel.y, rosNode.humanPosVel.vx, rosNode.humanPosVel.vy, 
                                 rosNode.robotPose.pose.pose.position.x, rosNode.robotPose.pose.pose.position.y, yawRobot, rosNode.semanticMapFrame, rosNode.semanticMapFrame);

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

            std::cout << "updateHypotheses, visualizations finished" << std::endl;    
            
            // WH: why wait?!
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
            // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0)); // Call ROS stream and wait 1 sec if no new measurement

            dt = ros::Time::now().toSec()-prevIt;
            updateKalman(A,H,P,Q,R,I,rosNode.humanPosVel,rosNode.measurement,dt);
            cout<<"dt = "<<dt<<endl;
            prevIt = ros::Time::now().toSec();
            rosNode.publishHumanPV();
        } else {
            ros::spinOnce();
        }

        r.sleep();
    }

    return 0;     /// [loop end]
}