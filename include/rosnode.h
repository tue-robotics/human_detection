#ifndef HIP_ROSNODE_H_
#define HIP_ROSNODE_H_

#include <ros/ros.h>

// ROS- hip_msgs // TODO all these msgs required? 
#include <hip_msgs/line.h>
#include <hip_msgs/lines.h>
#include <hip_msgs/tube.h>
#include <hip_msgs/singleTube.h>
#include <hip_msgs/tubes.h>
#include <hip_msgs/tubesH.h>
#include <hip_msgs/hypothesis.h>
#include <hip_msgs/hypotheses.h>
#include <hip_msgs/Pose.h>
#include <hip_msgs/PoseVel.h>
#include <hip_msgs/PoseVels.h>
#include <hip_msgs/detection.h>
#include <hip_msgs/detections.h>

// ROS tf2
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "associationMatrix.h"
#include "KalmanFilter.h"

using namespace std;

#define MARKER_LIFETIME 2.0

class rosNode {
    public:
    rosNode();
        // matrix A(4,vector<double>(4));
        struct measurement
        {
          measurement(double x_, double y_, double time_) : x(x_), y(y_), time(time_) {}

          double x;
          double y;
          double time;
        };

        std::vector<KalmanFilter> humanFilters;

        vector<measurement> measurements;

        // double[2] measurement;
        double p1,p2,p3,p4;

        // Start ROS listener
        ros::NodeHandle n;

        ros::Publisher semantic_map;
        ros::Publisher dynamic_map;
        ros::Publisher measuredHumans;
        ros::Publisher virtualRobot;
        ros::Publisher humansState;
        ros::Publisher humanPV;
        ros::Publisher tubeTop;
        ros::Publisher tubeHTop;
        ros::Publisher hypothesesTop;
        ros::Publisher humansSpeed;

        ros::Subscriber subHuman;
        ros::Subscriber subRobotPose;

        tf2_ros::Buffer tfBuffer;
        //tf2_ros::TransformListener* pTfListener;//(tfBuffer);
        tf2_ros::TransformListener tfListener;//(tfBuffer);

        std::string robotName;
        geometry_msgs::PoseWithCovarianceStamped robotPose;

//        std::vector<hip_msgs::PoseVel> humansPosVel;
        hip_msgs::PoseVels humanPosVels;
        visualization_msgs::Marker deleteAllMarker;
        visualization_msgs::MarkerArray deleteAllMarkerArray;
        visualization_msgs::MarkerArray markerArrayWalls;
        visualization_msgs::MarkerArray markerArrayAoI;
        visualization_msgs::MarkerArray markerArrayStatic;
        visualization_msgs::MarkerArray markerArrayDynamic;
        hip_msgs::tubes globalTube;

        int a = 4; // Meaning of this variable?

        // rosparam
        int iMax;
        bool real;
        string semanticMapFrame;
  
        //functions
        void createLine(int i, double xL, double yL, double zL, double xR, double yR, double zR, double r, double g,
                        double b, double radius, double a, visualization_msgs::Marker &marker);

        void processMap();

        void publishMap();

        void publishHumanPV();

        void visualizeHumans();

        void publishTube();

        void visualizeMeasuredHumans();

        void initialize();

//        void updateFakeMeasurement(const hip_msgs::Pose& poseHuman);
        void updateFakeMeasurement(const hip_msgs::detection& humanPose);

        void updateRealMeasurements(const hip_msgs::detections& humanPoses);

        void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

        // void getMeasurement(vector<double> &z);

        // void setHumanPosVel(human_intention_prediction::PoseVel humanPosVel_);

        // void getHumanPosVel(human_intention_prediction::PoseVel &humanPosVel_);

        void setStaticMap(visualization_msgs::MarkerArray staticMap);

        void setDynamicMap(visualization_msgs::MarkerArray dynamicMap);

        void publishTube(hip_msgs::tubes tube, hip_msgs::tubesH);

        void publishHypotheses(hip_msgs::hypotheses hypotheses, std::string ns);

        void visualizeRobot();

        void removeDynamicMap();
};

#endif // HIP_ROSNODE_H_