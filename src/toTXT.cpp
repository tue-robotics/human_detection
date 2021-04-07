#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"

#include <hip_msgs/Pose.h>
#include <hip_msgs/PoseVel.h>
#include <ed_gui_server/objsPosVel.h>
#include <functionsDiscretizedMap.h>
#include <hip_msgs/hypothesis.h>
#include <hip_msgs/hypotheses.h>
#include <hip_msgs/detections.h>

using namespace std;

hip_msgs::hypotheses hypothesesList;
hip_msgs::PoseVel humanPosVel;

void update(const hip_msgs::hypotheses& hypo) {
    hypothesesList = hypo;
} 
void state(const hip_msgs::PoseVel& humanPosVelTemp) {
    humanPosVel = humanPosVelTemp;
}

/// [main]
int main(int argc, char** argv)
/// [main]
{

    ros::init(argc, argv, "HIP");
    ros::NodeHandle n;
    ros::Subscriber hypotheses,human;
    hypotheses = n.subscribe("/HIP/hypotheses",1000, update);
    human = n.subscribe("/HIP/trackedHuman",1000, state);
    std::ofstream file,fileR,fileL,fileC;
    ros::Time stamp;
    double time;

    file.open("hypothesesPAOILRC.csv");

    while(ros::ok())
    {      
        stamp = ros::Time::now();
        time = stamp.toNSec();
        cout<<setprecision(35)<<time<<endl;
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
        cout<<"newmeasurement, "<<humanPosVel.x<<endl;
        file<<setprecision(35)<<time;
        file<<","<<humanPosVel.x<<","<<humanPosVel.y<<","<<humanPosVel.vx<<","<<humanPosVel.vy;
        // file<<setprecision(10)<<time;
        for (int i = 0; i<hypothesesList.hypotheses.size(); i++) {
            file<<","<<hypothesesList.hypotheses[i].index;
                file<<","<<hypothesesList.hypotheses[i].p;
        }
        file<<"\n";
    }
    /// [loop end]
    file.close();
    return 0;
}