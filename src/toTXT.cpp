#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
//#include "tf/transform_listener.h"

#include <hip_msgs/Pose.h>
#include <hip_msgs/PoseVel.h>
#include <ed_gui_server/objsPosVel.h>
#include <functionsDiscretizedMap.h>
#include <hip_msgs/hypothesis.h>
#include <hip_msgs/hypotheses.h>
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

using namespace std;
tf2_ros::Buffer tfBuffer;

std::string storeDir = "/home/ropod/Pictures/testData/";

hip_msgs::hypotheses hypothesesList;
hip_msgs::PoseVel humanPosVel;
geometry_msgs::PoseWithCovarianceStamped robotPose;
string semanticMapFrame;

void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

void update(const hip_msgs::hypotheses& hypo) {
    hypothesesList = hypo;
} 
/*void state(const hip_msgs::PoseVel& humanPosVelTemp) {
    humanPosVel = humanPosVelTemp;
}*/



/// [main]
int main(int argc, char** argv)
/// [main]
{

    ros::init(argc, argv, "HIP");
    ros::NodeHandle n;
    ros::Subscriber hypotheses,human,subRobotPose;

//    tf2_ros::Buffer tfBuffer;
        //tf2_ros::TransformListener* pTfListener;//(tfBuffer);
   tf2_ros::TransformListener tfListener(tfBuffer);//{ };//(tfBuffer);
//        tfListener(tfBuffer){ };

    hypotheses = n.subscribe("/HIP/hypotheses",1000, update);
//    human = n.subscribe("/HIP/trackedHuman",1000, state);
    subRobotPose = n.subscribe("/ropod_tue_2/amcl_pose", 1000, &updateRobotPose);
    

    std::ofstream fileP0,fileP0R,fileP0L,fileP0C;
    std::ofstream fileP1,fileP1R,fileP1L,fileP1C;
    std::ofstream fileRobotPose;

    ros::Time stamp = ros::Time::now();
    double time = stamp.toSec();
    std::string timeString = std::to_string( (int) time);


    n.getParam("/human_intention_prediction/mapFrame", semanticMapFrame);

//    std::cout << "toText: time = " << time << " timestring = " << timeString << std::endl;

    fileP0.open(storeDir + "hypothesesP0AOILRC" + timeString + ".csv");
//std::cout <<"file open = " << fileP0.is_open() << std::endl;
    fileP0L.open(storeDir + "hypothesesP0AOIL" + timeString + ".csv");
    fileP0R.open(storeDir + "hypothesesP0AOIR" + timeString + ".csv");
    fileP0C.open(storeDir + "hypothesesP0AOIC" + timeString + ".csv");

    fileP1.open(storeDir + "hypothesesP1AOILRC" + timeString + ".csv");
    fileP1L.open(storeDir + "hypothesesP1AOIL" + timeString + ".csv");
    fileP1R.open(storeDir + "hypothesesP1AOIR" + timeString + ".csv");
    fileP1C.open(storeDir + "hypothesesP1AOIC" + timeString + ".csv");

    fileP1C.open(storeDir + "hypothesesP1AOIC" + timeString + ".csv");

    fileRobotPose.open(storeDir + "robotPose" + timeString + ".csv");

    double t0 = 0;

    while(ros::ok())
    {      
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
//std::cout << "toTxt test" << std::endl;
        int person;


            
        std::cout << "hypothesesList.ns = " << hypothesesList.ns << " hypothesesList.timestamp = " << hypothesesList.timestamp - t0 << std::endl;
        if(hypothesesList.ns == "Hypotheses_Human0")
        {
            person = 0;
            fileP0 << setprecision(35) << hypothesesList.timestamp;
            fileP0L << setprecision(35) << hypothesesList.timestamp;
            fileP0R << setprecision(35) << hypothesesList.timestamp;
            fileP0C << setprecision(35) << hypothesesList.timestamp;
        } else if (hypothesesList.ns == "Hypotheses_Human1") {
            person = 1;
            fileP1 << setprecision(35) << hypothesesList.timestamp;
            fileP1L << setprecision(35) << hypothesesList.timestamp;
            fileP1R << setprecision(35) << hypothesesList.timestamp;
            fileP1C << setprecision(35) << hypothesesList.timestamp;
        } else {
//            std::cout << "going to continue" << std::endl;
            continue;
        }

        if(person == 0)
        {
            // std::cout << "writing to p0" << std::endl;
            fileP0 << "," << hypothesesList.humanPosVel.x << "," 
                   << hypothesesList.humanPosVel.y << "," 
                   << hypothesesList.humanPosVel.vx << "," 
                   << hypothesesList.humanPosVel.vy;

            // std::cout << "Pos vel info p0= " << hypothesesList.humanPosVel.x << "," 
                //    << hypothesesList.humanPosVel.y << "," 
                //    << hypothesesList.humanPosVel.vx << "," 
                //    << hypothesesList.humanPosVel.vy;

            for (int i = 0; i < hypothesesList.hypotheses.size(); i++) 
            {
                fileP0 << "," << hypothesesList.hypotheses[i].index;
                fileP0 << "," << hypothesesList.hypotheses[i].p;

               // std::cout << "index = " << hypothesesList.hypotheses[i].index;
               // std::cout << " prob = " << hypothesesList.hypotheses[i].p;

                if(hypothesesList.hypotheses[i].pSub.size() > 0)
                {
                    fileP0L << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[0];
                    fileP0R << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[1];
                    fileP0C << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[2];

                    //std::cout << "index = " << hypothesesList.hypotheses[i].index;
                   // std::cout << " lrc prob = " << hypothesesList.hypotheses[i].pSub[0] << " "
                   //  << hypothesesList.hypotheses[i].pSub[1] << " " << hypothesesList.hypotheses[i].pSub[2] << std::endl;
                }
            }

            fileP0 << "\n";
            fileP0L << "\n";
            fileP0R << "\n";
            fileP0C << "\n";

        } else if (person == 1){
                   
            if(t0 == 0)
        {
            t0 = hypothesesList.timestamp;
        }
//            std::cout << "writing to p1" << std::endl;
            fileP1 << "," << hypothesesList.humanPosVel.x << "," 
                   << hypothesesList.humanPosVel.y << "," 
                   << hypothesesList.humanPosVel.vx << "," 
                   << hypothesesList.humanPosVel.vy;

/*          std::cout << "Pos vel info p1= " << hypothesesList.humanPosVel.x << "," 
                   << hypothesesList.humanPosVel.y << "," 
                   << hypothesesList.humanPosVel.vx << "," 
                   << hypothesesList.humanPosVel.vy;
*/

            for (int i = 0; i < hypothesesList.hypotheses.size(); i++) 
            {
                fileP1 << "," << hypothesesList.hypotheses[i].index;
                fileP1 << "," << hypothesesList.hypotheses[i].p;

                std::cout << "index = " << hypothesesList.hypotheses[i].index;
                std::cout << " prob = " << hypothesesList.hypotheses[i].p;

                if(hypothesesList.hypotheses[i].pSub.size() > 0)
                {
                    fileP1L << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[0];
                    fileP1R << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[1];
                    fileP1C << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[2];

                    std::cout << "index = " << hypothesesList.hypotheses[i].index;
                    std::cout << " lrc prob = " << hypothesesList.hypotheses[i].pSub[0] << " "
                     << hypothesesList.hypotheses[i].pSub[1] << " " << hypothesesList.hypotheses[i].pSub[2] << std::endl;
                }
            }

            fileP1 << "\n";
            fileP1L << "\n";
            fileP1R << "\n";
            fileP1C << "\n";
        }


            double xRobot = robotPose.pose.pose.position.x;
            double yRobot = robotPose.pose.pose.position.y;

            tf2::Quaternion q ( robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, 
                                robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w );
            tf2::Matrix3x3 matrix ( q );
            double rollRobot, pitchRobot, yawRobot;
            matrix.getRPY ( rollRobot, pitchRobot, yawRobot );

            fileRobotPose << setprecision(35)<< robotPose.header.stamp << ", " << xRobot << ", " << yRobot << ", " << yawRobot << "\n";
    }

    fileP0.close();
    fileP0R.close();
    fileP0L.close();
    fileP0C.close();

    fileP1.close();
    fileP1R.close();
    fileP1L.close();
    fileP1C.close();

    return 0;
}

void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{   geometry_msgs::TransformStamped transformStamped;
    try{
        string updatedFrameID = msg.header.frame_id;
        // Check if frame-names start with a "/", as this is not allowed in order to determine the transformations
        if (updatedFrameID.substr(0,1) == "/") {
            updatedFrameID = updatedFrameID.substr (1,updatedFrameID.length()-1);
        }

        string updatedSemanticFrameID = semanticMapFrame;
        if (updatedSemanticFrameID.substr(0,1) == "/") {
            updatedSemanticFrameID = updatedSemanticFrameID.substr (1,updatedSemanticFrameID.length()-1);
        }

//std::cout <<"updateREalMeasurements,targer_frame updatedSemanticFrameID in updaterRobotpose = " << updatedSemanticFrameID << std::endl;
        transformStamped = tfBuffer.lookupTransform(updatedSemanticFrameID, updatedFrameID, ros::Time(0));
//std::cout << "rosnode, update robot pose, doTransform robotpose frame id = " << robotPose.header.frame_id << std::endl;
        tf2::doTransform(msg, robotPose, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s %s %d",ex.what(), __FILE__, __LINE__);
      ros::Duration(1.0).sleep();
//      continue;
    }
}
