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

std::string storeDir = "/home/ropod/Pictures/testData/";

hip_msgs::hypotheses hypothesesList;
hip_msgs::PoseVel humanPosVel;

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
    ros::Subscriber hypotheses,human;
    hypotheses = n.subscribe("/HIP/hypotheses",1000, update);
//    human = n.subscribe("/HIP/trackedHuman",1000, state);

    std::ofstream fileP0,fileP0R,fileP0L,fileP0C;
    std::ofstream fileP1,fileP1R,fileP1L,fileP1C;

    ros::Time stamp = ros::Time::now();
    double time = stamp.toSec();
    std::string timeString = std::to_string( (int) time);

  //  std::cout << "toText: time = " << time << " timestring = " << timeString << std::endl;

    fileP0.open(storeDir + "hypothesesP0AOILRC" + timeString + ".csv");
//std::cout <<"file open = " << fileP0.is_open() << std::endl;
    fileP0L.open(storeDir + "hypothesesP0AOIL" + timeString + ".csv");
    fileP0R.open(storeDir + "hypothesesP0AOIR" + timeString + ".csv");
    fileP0C.open(storeDir + "hypothesesP0AOIC" + timeString + ".csv");

    fileP1.open(storeDir + "hypothesesP1AOILRC" + timeString + ".csv");
    fileP1L.open(storeDir + "hypothesesP1AOIL" + timeString + ".csv");
    fileP1R.open(storeDir + "hypothesesP1AOIR" + timeString + ".csv");
    fileP1C.open(storeDir + "hypothesesP1AOIC" + timeString + ".csv");

    while(ros::ok())
    {      
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1000.0)); // Call ROS stream and wait 1000 sec if no new measurement
//std::cout << "toTxt test" << std::endl;
        int person;
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
            continue;
        }

        if(person == 0)
        {
            fileP0 << "," << hypothesesList.humanPosVel.x << "," 
                   << hypothesesList.humanPosVel.y << "," 
                   << hypothesesList.humanPosVel.vx << "," 
                   << hypothesesList.humanPosVel.vy;

            for (int i = 0; i < hypothesesList.hypotheses.size(); i++) 
            {
                fileP0 << "," << hypothesesList.hypotheses[i].index;
                fileP0 << "," << hypothesesList.hypotheses[i].p;

                if(hypothesesList.hypotheses[i].pSub.size() > 0)
                {
                    fileP0L << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[0];
                    fileP0R << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[1];
                    fileP0C << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[2];
                }
            }

            fileP0 << "\n";
            fileP0L << "\n";
            fileP0R << "\n";
            fileP0C << "\n";

        } else {
            fileP1 << "," << hypothesesList.humanPosVel.x << "," 
                   << hypothesesList.humanPosVel.y << "," 
                   << hypothesesList.humanPosVel.vx << "," 
                   << hypothesesList.humanPosVel.vy;

            for (int i = 0; i < hypothesesList.hypotheses.size(); i++) 
            {
                fileP1 << "," << hypothesesList.hypotheses[i].index;
                fileP1 << "," << hypothesesList.hypotheses[i].p;

                if(hypothesesList.hypotheses[i].pSub.size() > 0)
                {
                    fileP1L << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[0];
                    fileP1R << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[1];
                    fileP1C << "," << hypothesesList.hypotheses[i].index << "," << hypothesesList.hypotheses[i].pSub[2];
                }
            }

            fileP1 << "\n";
            fileP1L << "\n";
            fileP1R << "\n";
            fileP1C << "\n";
        }

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
