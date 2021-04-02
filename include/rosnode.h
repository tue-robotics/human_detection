#ifndef HIP_ROSNODE_H_
#define HIP_ROSNODE_H_

#include <human_intention_prediction/Pose.h>
#include <human_intention_prediction/PoseVel.h>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf/transform_listener.h>
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

   //#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <camera_detector/detection.h>
#include <camera_detector/detections.h>

// ROS-msgs
#include <human_intention_prediction/line.h>
#include <human_intention_prediction/lines.h>
#include <human_intention_prediction/tube.h>
#include <human_intention_prediction/singleTube.h>
#include <human_intention_prediction/tubes.h>
#include <human_intention_prediction/tubesH.h>
#include <human_intention_prediction/hypothesis.h>
#include <human_intention_prediction/hypotheses.h>

using namespace std;

class rosNode {     
    public:
    rosNode();
        // matrix A(4,vector<double>(4));

        vector<double> measurement;

        // double[2] measurement;
        double p1,p2,p3,p4;

        // Start ROS listener
        ros::NodeHandle n;

        ros::Publisher semantic_map;
        ros::Publisher dynamic_map;
        ros::Publisher measuredHuman;
        ros::Publisher virtualRobot;
        ros::Publisher humanState;
        ros::Publisher humanPV;
        ros::Publisher tubeTop;
        ros::Publisher tubeHTop;
        ros::Publisher hypothesesTop;
        ros::Publisher humanSpeed;

        ros::Subscriber subHuman;
        ros::Subscriber subRobotPose;

        tf2_ros::Buffer tfBuffer;
        //tf2_ros::TransformListener* pTfListener;//(tfBuffer);
         tf2_ros::TransformListener tfListener;//(tfBuffer);

        std::string robotName;
        geometry_msgs::PoseWithCovarianceStamped robotPose;

        human_intention_prediction::PoseVel humanPosVel;
        visualization_msgs::Marker deleteAllMarker;
        visualization_msgs::MarkerArray deleteAllMarkerArray;
        visualization_msgs::MarkerArray markerArrayWalls;
        visualization_msgs::MarkerArray markerArrayAoI;
        visualization_msgs::MarkerArray markerArrayStatic;
        visualization_msgs::MarkerArray markerArrayDynamic;
        visualization_msgs::Marker markerA;
        human_intention_prediction::tubes globalTube;

        int a=4;

        // rosparam
        int iMax;
        bool real;
        string semanticMapFrame;
        //double xRobot,yRobot,thetaRobot;
  
        //functions
        void createLine(int i, double xL, double yL, double zL, double xR, double yR, double zR, double r, double g, double b, double radius, double a, visualization_msgs::Marker &marker);

        void processMap();

        void publishMap();

        void publishHumanPV();

        void visualizeHuman();

        void publishTube();

        void visualizeMeasuredHuman();

        void initialize();

        void updateFakeMeasurement(const human_intention_prediction::Pose& poseHuman);

        void updateRealMeasurement(const camera_detector::detections& poseHuman);

        void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

        // void getMeasurement(vector<double> &z);

        // void setHumanPosVel(human_intention_prediction::PoseVel humanPosVel_);

        // void getHumanPosVel(human_intention_prediction::PoseVel &humanPosVel_);

        void setStaticMap(visualization_msgs::MarkerArray staticMap);

        void setDynamicMap(visualization_msgs::MarkerArray dynamicMap);

        void publishTube(human_intention_prediction::tubes tube, human_intention_prediction::tubesH);

        void publishHypotheses(human_intention_prediction::hypotheses hypotheses);

        void visualizeRobot();

        void removeDynamicMap();
};

#endif // HIP_ROSNODE_H_