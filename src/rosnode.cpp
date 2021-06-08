#include <rosnode.h>

rosNode::rosNode(void): tfListener(tfBuffer){ };

void rosNode::initialize() {
    cout<<"Starting ROS publisher..."<<endl;
    measuredHumans = n.advertise<visualization_msgs::MarkerArray>("/HIP/measuredHumans",3);
    virtualRobot = n.advertise<visualization_msgs::Marker>("/HIP/virtualRobot",3);
    humansState = n.advertise<visualization_msgs::MarkerArray>("/HIP/humansState",3);
    humansSpeed = n.advertise<visualization_msgs::MarkerArray>("/HIP/humansSpeed",3);
    humanPV = n.advertise<hip_msgs::PoseVels>("/HIP/trackedHumans",3);
    tubeTop = n.advertise<hip_msgs::tubes>("/HIP/tubes",3);
    tubeHTop = n.advertise<hip_msgs::tubesH>("/HIP/tubesH",3);
    hypothesesTop = n.advertise<hip_msgs::hypotheses>("/HIP/hypotheses",3);
    deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;

    processingTimeTop = n.advertise<std_msgs::Duration>("/HIP/ProcessingTime",3);

    n.getParam("/human_intention_prediction/a",iMax);
    n.getParam("/human_intention_prediction/real",real);
    n.getParam("/human_intention_prediction/robotName",robotName);
    n.getParam("/human_intention_prediction/mapFrame", semanticMapFrame);

    subRobotPose = n.subscribe(robotName + "/amcl_pose", 1000, &rosNode::updateRobotPose, this);
    
//    if (real) {
        subHuman = n.subscribe("/Jetson/cameraDetections", 1000, &rosNode::updateRealMeasurements, this);
//    } else {
//        subHuman = n.subscribe("/Human/pose", 1000, &rosNode::updateFakeMeasurement, this);
//    }

    processMap();
}

void rosNode::createLine(   int i,                                  // ID
                            double xL, double yL, double zL,        // points low
                            double xR, double yR, double zR,        // points high
                            double r, double g, double b,           // rgb-values of color
                            double radius, double a,                
                            visualization_msgs::Marker &marker) {   // specified marker
    double dx,dy,dz, dist;
    double roll, pitch, yaw;

    dx = xL-xR;
    dy = yL-yR;
    dz = zL-zR;

    dist = sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));

    roll = atan2(sqrt(pow(dx,2.0)+pow(dy,2.0)),dz);
    pitch = atan2(-dy,dx); //atan2(dy,dz)
    yaw = 0.0;
    tf2::Quaternion angleQ;
    angleQ.setEuler(roll,pitch,yaw);
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (xL+xR)/2.0;
    marker.pose.position.y = (yL+yR)/2.0;
    marker.pose.position.z = (zL+zR)/2.0;

    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = dist;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    marker.lifetime = ros::Duration(MARKER_LIFETIME);
}

/*void rosNode::updateFakeMeasurement(const hip_msgs::detection& humanPose) {
    measurements.clear();

    measurement meas(humanPose.x, humanPose.y);
    measurements.push_back(meas);
}*/

void rosNode::updateRealMeasurements(const hip_msgs::detections& humanPoses) {

//std:cout << "updateRealMeasurements, humanPoses.size() = " << humanPoses.detections.size() << std::endl;

    if (humanPoses.detections.size() > 0) {
        try
        {
            measurements.clear();
            for(int i = 0; i < humanPoses.detections.size(); i++)
            {
                hip_msgs::detection humanPose = humanPoses.detections[i];
                string measuredFrameID = humanPoses.header.frame_id;
                
                // Check if frame-names start with a "/", as this is not allowed in order to determine the transformations
                if (measuredFrameID.substr(0,1) == "/") 
                {
                    measuredFrameID = measuredFrameID.substr (1,measuredFrameID.length()-1);
                }

                string updatedSemanticFrameID = semanticMapFrame;
                if (updatedSemanticFrameID.substr(0,1) == "/") 
                {
                    updatedSemanticFrameID = updatedSemanticFrameID.substr (1,updatedSemanticFrameID.length()-1);
                }

                geometry_msgs::Point pointHumanCameraFrame, pointHumanMapFrame;
//std::cout <<"updateREalMeasurements,targer_frame updatedSemanticFrameID = " << updatedSemanticFrameID << std::endl;
                geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(updatedSemanticFrameID, measuredFrameID, ros::Time(0));

                pointHumanCameraFrame.x = humanPose.x;
                pointHumanCameraFrame.y = humanPose.y;
                pointHumanCameraFrame.z = humanPose.z;
//std::cout << "updateRealMeasurements: updatedSemanticFrameID = " << updatedSemanticFrameID << " measuredFrameID = " << measuredFrameID << std::endl;
			tf2::doTransform(pointHumanCameraFrame, pointHumanMapFrame, transformStamped);


                double tMeasurement = humanPoses.header.stamp.sec + humanPoses.header.stamp.nsec/1e9;
//std::cout << "updateRealMeasurement: tMeasurement = " << tMeasurement << " stamp.sec = " << humanPoses.header.stamp.sec << " stamp.nsec = " << humanPoses.header.stamp.nsec << std::endl;
                measurement meas(pointHumanMapFrame.x, pointHumanMapFrame.y, tMeasurement);
                measurements.push_back(meas);
            }

            HIP::AssociationMatrix assoc_matrix(measurements.size()); // TODO! -> process measurements in KF

            for (unsigned int iMeas = 0; iMeas < measurements.size(); ++iMeas)
            {
                measurement meas = measurements[iMeas];

                for (unsigned int iHumans = 0; iHumans < humanFilters.size(); ++iHumans)
                {
//                    const ed::EntityConstPtr& e = entities[i_entity];
                    KalmanFilter human = humanFilters[iHumans]; // TODO add prediction step based on constant velocity model

		    //std::cout << "updateRealMeasurement, dt = " << meas.time - human.getLatestUpdateTime() << std::endl;
                    hip_msgs::PoseVel predictedPos =  human.predictPos(meas.time);

//std::cout << "statePos = " << human.toString() << std::endl;
//std::cout << "predictedPos = " << predictedPos.x << ", " << predictedPos.y << std::endl;
                    //const geo::Pose3D& entity_pose = e->pose();
//                    const ed::ConvexHull& entity_chull = e->convexHull();

                    float dx = predictedPos.x - meas.x;
                    float dy = predictedPos.y - meas.y;
                    float dz = 0;

//		std::cout << "updateRealMeasurement, dx, dy, dz = " << dx << ", " << dy << ", " << dz << std::endl;

//                    if (entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose.t.z
//                            || cluster.chull.z_max + cluster.pose.t.z < entity_chull.z_min + entity_pose.t.z)
                        // The convex hulls are non-overlapping in z
//                        dz = entity_pose.t.z - cluster.pose.t.z;

                    float dist_sq = (dx * dx + dy * dy + dz * dz); // TODO take Mahalanobis distance as criterion?!
//std::cout << "updateRealMeasurement, dist = " << sqrt(dist_sq) << std::endl;

                    // TODO: better prob calculation
                    double prob = 1.0 / (1.0 + 100 * dist_sq);

                    double dt = meas.time - human.getLatestUpdateTime();
                    double e_max_dist = std::max(0.2, std::min(0.5, dt * 10));

//std::cout << "updateRealMeasurement, e_max_dist = " << e_max_dist << std::endl;

                    if (dist_sq > e_max_dist * e_max_dist)
                        prob = 0;

                    if (prob > 0)
                        assoc_matrix.setEntry(iMeas, iHumans, prob);
                }
            }

            HIP::Assignment assig;
            if (!assoc_matrix.calculateBestAssignment(assig))
            {
                std::cout << "WARNING: Association failed!" << std::endl;
                return;
            }

            std::vector<int> objectsAssociated(humanFilters.size(), -1);

            for (unsigned int iMeas = 0; iMeas < measurements.size(); ++iMeas)
            {
                measurement meas = measurements[iMeas];

                // Get the assignment for this cluster
                int iHuman = assig[iMeas];

                if (iHuman == -1)
                {
                    // No assignment, so create new object
                    KalmanFilter human;
                    human.init(meas.time, meas.x, meas.y);

//		    std::cout << "updateREalMeasurement, before add new filter, humanFilters.size() = " << humanFilters.size() << std::endl;
                    humanFilters.push_back(human);
//		    std::cout << "updateREalMeasurement, KF init, state = " << human.toString();
//		    std::cout << "updateREalMeasurement, afer add new filter, humanFilters.size() = " << humanFilters.size() << std::endl;
                }
                else
                {
                    // Mark the entity as being associated
                    objectsAssociated[iHuman] = iMeas;

                    // Update the human pose
                    std::vector<double> measurementVector;
                    measurementVector.push_back(meas.x);
                    measurementVector.push_back(meas.y);

                    hip_msgs::PoseVel updatedHumanPosVel; // TODO where to store?
                    humanFilters[iHuman].update(updatedHumanPosVel, measurementVector, meas.time);
//		    std::cout << "updateRealMeasurement, KF updated, state = " << humanFilters[iHuman].toString();
                }
            }
        }

        catch (tf2::TransformException &ex) 
        {
        
//std::cout <<"updateRealMeasurements, try failed" << std::endl;
    ROS_WARN("%s %s %d",ex.what(), __FILE__, __LINE__);
            ros::Duration(1.0).sleep();
        }
    }

//    std::cout << "updateRealMeasurement, human filters size before erasing filters = " << humanFilters.size() << std::endl;
    ros::Time currentTime = ros::Time::now();
    double curTime = currentTime.toSec();
    for(int iHuman = 0; iHuman < humanFilters.size(); )
    {
//std::cout << "updateRealMeasurement, curTime = " << curTime << ", filtertime = " << humanFilters[iHuman].getLatestUpdateTime() << std::endl;
        if(curTime - humanFilters[iHuman].getLatestUpdateTime() > 3.0)
        {
            humanFilters.erase(humanFilters.begin() + iHuman);
        } else {
            iHuman++;
        }
    }
}

void rosNode::updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
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

void rosNode::processMap() {
/// [plot map]
// create markers for visualization
    visualization_msgs::Marker marker;
    marker.ns = "wall";
    marker.header.frame_id = semanticMapFrame;
    marker.action = visualization_msgs::Marker::ADD;
    //semantic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/semanticMap",3);
    semantic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/" + semanticMapFrame,3);
    dynamic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/dynamicMap",3);
// obtain already published map information from yaml file
    XmlRpc::XmlRpcValue walls,AoI;
    n.getParam("/human_intention_prediction/walls",walls);
    n.getParam("/human_intention_prediction/AoI",AoI);
// plot walls
    for (int i=0;i<walls.size();i++) {
        createLine(i,walls[i]["x0"],walls[i]["y0"],walls[i]["z0"],walls[i]["x1"],walls[i]["y1"],walls[i]["z1"],1.0,0.0,0.0,0.05,0.4,marker);
        markerArrayWalls.markers.push_back(marker);
    }
// plot Areas of Interest
    marker.ns = "AoI";
    for (int i=0;i<AoI.size();i++) {
        if (i==3&&i==4&&i==8) {
            createLine(i,AoI[i]["x0"],AoI[i]["y0"],AoI[i]["z0"],AoI[i]["x1"],AoI[i]["y1"],AoI[i]["z1"],0.0,0.3,0.0,0.05,0.1,marker);
            markerArrayAoI.markers.push_back(marker);
        } else {
            createLine(i,AoI[i]["x0"],AoI[i]["y0"],AoI[i]["z0"],AoI[i]["x1"],AoI[i]["y1"],AoI[i]["z1"],0.3,0.0,0.0,0.05,0.1,marker);
            markerArrayAoI.markers.push_back(marker);
        }
    }
/// [plot map]
}

void rosNode::publishMap() {
    semantic_map.publish (markerArrayAoI);
    semantic_map.publish (markerArrayWalls);
    semantic_map.publish (markerArrayStatic);
    dynamic_map.publish (markerArrayDynamic);
}

void rosNode::removeDynamicMap() {
    deleteAllMarkerArray.markers.clear();
    // for (int i=0;i<markerArrayDynamic.markers.size();i++) {
        // deleteAllMarker = markerArrayDynamic.markers[i];
        // deleteAllMarker.action = visualization_msgs::Marker::DELETE;
    deleteAllMarkerArray.markers.push_back(deleteAllMarker);
    // }
    // deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
    dynamic_map.publish (deleteAllMarkerArray);
}

void rosNode::publishTube(hip_msgs::tubes tube, hip_msgs::tubesH tubesH) {
    tubeTop.publish(tube);
    tubeHTop.publish(tubesH);
}

void rosNode::publishHypotheses(hip_msgs::hypotheses hypotheses, std::string ns, KalmanFilter &humanFilter) {
    hypotheses.timestamp = humanFilter.getLatestUpdateTime();

    hypotheses.humanPosVel = humanFilter.predictPos(humanFilter.getLatestUpdateTime() );

    hypotheses.ns = ns;
    hypothesesTop.publish(hypotheses);
}

void rosNode::publishHumanPV() {
    humanPV.publish (humanPosVels);
}

void rosNode::publishProcessingTime(ros::Duration dt) {

    std_msgs::Duration dt_msgs;
    dt_msgs.data = dt;

    processingTimeTop.publish (dt_msgs);
}


void rosNode::visualizeHumans() {
 
    visualization_msgs::MarkerArray currentHumansState, currentHumansSpeed;
    hip_msgs::PoseVels humanPosVelsNew;//[humanFilters.size()];

//	std::cout << "humanFilters, size() = " << humanFilters.size() << std::endl;    

for(unsigned int i = 0; i < humanFilters.size(); i++)
    {
        visualization_msgs::Marker humanState;
        humanState.ns = "humanState";
//        humanState.id = i + 1;

//        std::cout << "visualizeHumans, i = " << i << " humanState.id = " << humanState.id << std::endl;

        hip_msgs::PoseVel humanPosVel = humanFilters[i].predictPos(humanFilters[i].getLatestUpdateTime());
        humanPosVelsNew.poseVels.push_back(humanPosVel);
//std::cout << "rosnode, visualize humans, pos = " << humanPosVel.x << ", " << humanPosVel.y << std::endl;
        createLine(i,humanPosVel.x, humanPosVel.y, 0.0, humanPosVel.x, humanPosVel.y, 1.8, 0.0, 0.0, 1.0, 0.3, 1.0, humanState);
        humanState.action = 0; //# 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
        humanState.header.frame_id = semanticMapFrame;
	humanState.lifetime = ros::Duration(2.0);
        currentHumansState.markers.push_back(humanState);
        //humanState.publish (deleteAllMarker);

        visualization_msgs::Marker markerSpeed;
        geometry_msgs::Point sample;
        sample.x  = humanPosVel.x;
        sample.y = humanPosVel.y;
        sample.z = 0.0;
        markerSpeed.points.push_back(sample);
        sample.x = humanPosVel.x + humanPosVel.vx;
        sample.y = humanPosVel.y + humanPosVel.vy;
        sample.z = 0.0;

        markerSpeed.points.push_back(sample);
        markerSpeed.type = visualization_msgs::Marker::ARROW;
        markerSpeed.header.stamp = ros::Time();

        markerSpeed.scale.x = 0.07;
        markerSpeed.scale.y = 0.1;
        markerSpeed.color.a = 1;
        markerSpeed.color.r = 0;
        markerSpeed.color.g = 0;
        markerSpeed.color.b = 1;
        markerSpeed.ns = "wall";
        markerSpeed.id = i + 1;
        markerSpeed.header.frame_id = semanticMapFrame;
        markerSpeed.lifetime = ros::Duration(MARKER_LIFETIME);

//        std::cout << "visualizeHumans, i = " << i << " markerSpeed.id = " << markerSpeed.id << std::endl;

        currentHumansSpeed.markers.push_back(markerSpeed);
    }

    humansState.publish(currentHumansState);
    humansSpeed.publish(currentHumansSpeed);
    
//    hip_msgs::PoseVels humanPosVelsUpdated;
//    humanPosVelsUpdated.poseVels = humanPosVelsNew;
    humanPosVels = humanPosVelsNew;
}

void rosNode::visualizeMeasuredHumans() {
//    measuredHuman.publish (deleteAllMarker);
    visualization_msgs::MarkerArray humanMarkers;

    for(unsigned int i = 0; i < measurements.size(); i++)
    {
        visualization_msgs::Marker humanMarker;
        humanMarker.ns = "measuredHuman";
        humanMarker.header.frame_id = semanticMapFrame;
        humanMarker.action = visualization_msgs::Marker::ADD; //# 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects

        createLine(i,measurements[i].x, measurements[i].y, 0.0, measurements[i].x, measurements[i].y, 1.8, 0.5, 0.5, 0.5, 0.3, 1.0, humanMarker);
        humanMarkers.markers.push_back(humanMarker);
    }

    measuredHumans.publish (humanMarkers);
}

void rosNode::visualizeRobot() {
    visualization_msgs::Marker marker;
    marker.ns = "virtualRobot";
    marker.header.frame_id = semanticMapFrame;

    double xRobot = robotPose.pose.pose.position.x;
    double yRobot = robotPose.pose.pose.position.y;

    tf2::Quaternion q ( robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, 
                        robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w );
    tf2::Matrix3x3 matrix ( q );
    double rollRobot, pitchRobot, yawRobot;
    matrix.getRPY ( rollRobot, pitchRobot, yawRobot );

    int ID = 1;
    double xL = xRobot, yL = yRobot, zL = 0.0;
    double xR = xRobot, yR = yRobot, zR = 1.8;
    double r = 153, g = 0.0, b = 153, a = 1.0;
    double radius = 0.3;

     createLine(ID, xL, yL, zL, xR, yR, zR, r, g, b, radius, a,marker);
     virtualRobot.publish (deleteAllMarker);
     virtualRobot.publish (marker);
}

void rosNode::setStaticMap(visualization_msgs::MarkerArray staticMap) {
    markerArrayStatic = staticMap;
}

void rosNode::setDynamicMap(visualization_msgs::MarkerArray dynamicMap) {
    markerArrayDynamic = dynamicMap;
}
