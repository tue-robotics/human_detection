#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

#include <hip_msgs/walls.h>
#include <hip_msgs/wall.h>
#include <hip_msgs/Pose.h>
#include <hip_msgs/hypothesis.h>
#include <hip_msgs/hypotheses.h>

#include <ed_gui_server/objsPosVel.h>
#include <functions.h>
#include <rosnode.h>
#include <functionsDiscretizedMap.h>

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
    
//    matrix A,H,P,Q,R,I; // Kalman filter matrices

//    initializeKalman(A,H,P,Q,R,I,1/rate);
    double tPrev = ros::Time::now().toSec();
//    double dt;

    double rate = 15.0;
    ros::Rate r(rate); // loop at 15 hz

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

        if(poseValid) // TODO apply for all humans, publish hypotheses on namespace with separate human ID
        {
            std::cout << "main, rosNode.humanFilters.size() = " << rosNode.humanFilters.size() << std::endl;

            for(unsigned int iHumans = 0; iHumans < rosNode.humanFilters.size(); iHumans++)
            {
                hip_msgs::PoseVel humanPosVel = rosNode.humanFilters[iHumans].predictPos(rosNode.humanFilters[iHumans].getLatestUpdateTime() );


                std::cout << "main, iHumans = " << iHumans << " humanPosVel.x = " << humanPosVel.x << " humanPosVel.y = " << humanPosVel.y << std::endl;

                map.updateHypotheses(humanPosVel.x, humanPosVel.y, humanPosVel.vx, humanPosVel.vy, 
                                    rosNode.robotPose.pose.pose.position.x, rosNode.robotPose.pose.pose.position.y, yawRobot, rosNode.semanticMapFrame,
                                    rosNode.semanticMapFrame);

                std::string ns = "Hypotheses_Human" +  std::to_string(iHumans);
                rosNode.publishHypotheses(map.hypotheses, ns);
                map.readMap(staticMarkers,dynamicMarkers);

                for(unsigned int iMarker = 0; iMarker < dynamicMarkers.markers.size(); iMarker++)
                {
                    visualization_msgs::Marker marker = dynamicMarkers.markers[iMarker];
                    marker.ns = marker.ns + "object" + std::to_string(iHumans);
                    dynamicMarkers.markers[iMarker] = marker;
                }
                
                if (i%3==0) 
                {
                    rosNode.removeDynamicMap();
                }
                rosNode.setDynamicMap(dynamicMarkers);
                rosNode.publishMap();
            }
            // }
            rosNode.visualizeHumans();
            rosNode.visualizeMeasuredHumans();
            rosNode.visualizeRobot();

            std::cout << "updateHypotheses, visualizations finished" << std::endl;    
            
            // WH: why wait?!
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
            // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0)); // Call ROS stream and wait 1 sec if no new measurement

//            double dt = ros::Time::now().toSec() - tPrev; //TODO use measurement time!
//            updateKalman(A,H,P,Q,R,I,rosNode.humanPosVel,rosNode.measurement,dt); // TODO -> for all humans
//            cout<<"dt = "<<dt<<endl;
//            tPrev = ros::Time::now().toSec();
            rosNode.publishHumanPV();
        } else {
            ros::spinOnce();
        }

        r.sleep();
    }

    return 0;     /// [loop end]
}