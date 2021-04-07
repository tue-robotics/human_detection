#include <rosnode.h>

rosNode::rosNode(void): tfListener(tfBuffer){ };

void rosNode::initialize() {
    cout<<"Starting ROS publisher..."<<endl;
    measuredHuman = n.advertise<visualization_msgs::Marker>("/HIP/measuredHuman",3);
    virtualRobot = n.advertise<visualization_msgs::Marker>("/HIP/virtualRobot",3);
    humanState = n.advertise<visualization_msgs::Marker>("/HIP/humanState",3);
    humanSpeed = n.advertise<visualization_msgs::Marker>("/HIP/humanSpeed",3);
    humanPV = n.advertise<hip_msgs::PoseVel>("/HIP/trackedHuman",3);
    tubeTop = n.advertise<hip_msgs::tubes>("/HIP/tubes",3);
    tubeHTop = n.advertise<hip_msgs::tubesH>("/HIP/tubesH",3);
    hypothesesTop = n.advertise<hip_msgs::hypotheses>("/HIP/hypotheses",3);
    deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;

    n.getParam("/human_intention_prediction/a",iMax);
    n.getParam("/human_intention_prediction/real",real);
    n.getParam("/human_intention_prediction/robotName",robotName);
    n.getParam("/human_intention_prediction/mapFrame", semanticMapFrame);

    subRobotPose = n.subscribe(robotName + "/amcl_pose", 1000, &rosNode::updateRobotPose, this);
    
    if (real) {
        subHuman = n.subscribe("/Jetson/cameraDetections", 1000, &rosNode::updateRealMeasurement, this);
    } else {
        subHuman = n.subscribe("/Human/pose", 1000, &rosNode::updateFakeMeasurement, this);
    }

    processMap();
    humanPosVel.x = 0.0;
    humanPosVel.y = 0.0;
    humanPosVel.vx = 0.0;
    humanPosVel.vy = 0.0;
    measurement.push_back(0.0);
    measurement.push_back(0.0);
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
}

void rosNode::updateFakeMeasurement(const hip_msgs::Pose& poseHuman) {
    measurement[0] = poseHuman.x;
    measurement[1] = poseHuman.y;
}

void rosNode::updateRealMeasurement(const hip_msgs::detections& poseHuman) {
    if (poseHuman.detections.size()>0) {

        double xRobot = robotPose.pose.pose.position.x;
        double yRobot = robotPose.pose.pose.position.y;

        tf2::Quaternion q ( robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, 
                            robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w );
        tf2::Matrix3x3 matrix ( q );
        double rollRobot, pitchRobot, yawRobot;
        matrix.getRPY ( rollRobot, pitchRobot, yawRobot );

        // here, only first detection is considered(!)
        measurement[0] = cos(yawRobot) * (poseHuman.detections[0].x) - sin(yawRobot) * poseHuman.detections[0].y + xRobot;
        measurement[1] = sin(yawRobot) * (poseHuman.detections[0].x) + cos(yawRobot) * poseHuman.detections[0].y + yRobot;
    }
}

void rosNode::updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{    geometry_msgs::TransformStamped transformStamped;
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

        transformStamped = tfBuffer.lookupTransform(updatedSemanticFrameID, updatedFrameID, ros::Time(0));
        tf2::doTransform(msg, robotPose, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
//      continue;
    }
}

void rosNode::processMap() {
/// [plot map]
// create markers for visualization
    markerA.ns = "wall";
    markerA.header.frame_id = semanticMapFrame;
    markerA.action = visualization_msgs::Marker::ADD;
    //semantic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/semanticMap",3);
    semantic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/" + semanticMapFrame,3);
    dynamic_map = n.advertise<visualization_msgs::MarkerArray>("/HIP/dynamicMap",3);
// obtain already published map information from yaml file
    XmlRpc::XmlRpcValue walls,AoI;
    n.getParam("/human_intention_prediction/walls",walls);
    n.getParam("/human_intention_prediction/AoI",AoI);
// plot walls
    for (int i=0;i<walls.size();i++) {
        createLine(i,walls[i]["x0"],walls[i]["y0"],walls[i]["z0"],walls[i]["x1"],walls[i]["y1"],walls[i]["z1"],1.0,0.0,0.0,0.05,0.4,markerA);
        markerArrayWalls.markers.push_back(markerA);
    }
// plot Areas of Interest
    markerA.ns = "AoI";
    for (int i=0;i<AoI.size();i++) {
        if (i==3&&i==4&&i==8) {
            createLine(i,AoI[i]["x0"],AoI[i]["y0"],AoI[i]["z0"],AoI[i]["x1"],AoI[i]["y1"],AoI[i]["z1"],0.0,0.3,0.0,0.05,0.1,markerA);
            markerArrayAoI.markers.push_back(markerA);
        } else {
            createLine(i,AoI[i]["x0"],AoI[i]["y0"],AoI[i]["z0"],AoI[i]["x1"],AoI[i]["y1"],AoI[i]["z1"],0.3,0.0,0.0,0.05,0.1,markerA);
            markerArrayAoI.markers.push_back(markerA);
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

void rosNode::publishHypotheses(hip_msgs::hypotheses hypotheses) {
    hypothesesTop.publish(hypotheses);
}

void rosNode::publishHumanPV() {
    humanPV.publish (humanPosVel);
}

void rosNode::visualizeHuman() {
    markerA.ns = "humanState";
    createLine(1,humanPosVel.x,humanPosVel.y,0.0,humanPosVel.x,humanPosVel.y,1.8,0.0,0.0,1.0,0.3,1.0,markerA);
    humanState.publish (deleteAllMarker);
    humanState.publish (markerA);
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
    markerSpeed.header.frame_id = semanticMapFrame;
    humanSpeed.publish(markerSpeed);
}

void rosNode::visualizeMeasuredHuman() {
    markerA.ns = "measuredHuman";
    markerA.header.frame_id = semanticMapFrame;

    createLine(1,measurement[0],measurement[1],0.0,measurement[0],measurement[1],1.8,0.5,0.5,0.5,0.3,1.0,markerA);
    measuredHuman.publish (deleteAllMarker);
    measuredHuman.publish (markerA);
}

void rosNode::visualizeRobot() {
    markerA.ns = "virtualRobot";
    markerA.header.frame_id = semanticMapFrame;

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

     createLine(ID, xL, yL, zL, xR, yR, zR, r, g, b, radius, a,markerA);
     virtualRobot.publish (deleteAllMarker);
     virtualRobot.publish (markerA);
}

void rosNode::setStaticMap(visualization_msgs::MarkerArray staticMap) {
    markerArrayStatic = staticMap;
}

void rosNode::setDynamicMap(visualization_msgs::MarkerArray dynamicMap) {
    markerArrayDynamic = dynamicMap;
}