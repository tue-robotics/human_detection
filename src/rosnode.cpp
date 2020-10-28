#include <rosnode.h>


void rosNode::initialize() {
    cout<<"Starting ROS publisher..."<<endl;
    measuredHuman = n.advertise<visualization_msgs::Marker>("/HWD/measuredHuman",3);
    virtualRobot = n.advertise<visualization_msgs::Marker>("/HWD/virtualRobot",3);
    humanState = n.advertise<visualization_msgs::Marker>("/HWD/humanState",3);
    humanSpeed = n.advertise<visualization_msgs::Marker>("/HWD/humanSpeed",3);
    humanPV = n.advertise<human_walking_detection::PoseVel>("/HWD/trackedHuman",3);
    tubeTop = n.advertise<human_walking_detection::tubes>("/HWD/tubes",3);
    tubeHTop = n.advertise<human_walking_detection::tubesH>("/HWD/tubesH",3);
    hypothesesTop = n.advertise<human_walking_detection::hypotheses>("/HWD/hypotheses",3);
    deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
    n.getParam("/human_walking_detection/a",iMax);
    n.getParam("/human_walking_detection/real",real);
    n.getParam("/human_walking_detection/xRobot",xRobot);
    n.getParam("/human_walking_detection/yRobot",yRobot);
    n.getParam("/human_walking_detection/thetaRobot",thetaRobot);
    if (real) {
        subHuman = n.subscribe("/Jetson/cameraDetections",1000, &rosNode::updateRealMeasurement, this);
    } else {
        subHuman = n.subscribe("/virtualHuman/pose",1000, &rosNode::updateFakeMeasurement, this);
    }   
    processMap();
    humanPosVel.x = 0.0;
    humanPosVel.y = 0.0;
    humanPosVel.vx = 0.0;
    humanPosVel.vy = 0.0;
    measurement.push_back(0.0);
    measurement.push_back(0.0);
}

void rosNode::createLine(int i, double xL, double yL, double zL, double xR, double yR, double zR, double r, double g, double b, double radius, double a, visualization_msgs::Marker &marker) {
    double dx,dy,dz, dist;
    double t3,t2,t1;

    dx = xL-xR;
    dy = yL-yR;
    dz = zL-zR;

    dist = sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));

    t1 = atan2(-dy,dx); //atan2(dy,dz)
    t2 = atan2(sqrt(pow(dx,2.0)+pow(dy,2.0)),dz);
    t3 = 0.0;
    tf2::Quaternion angleQ;
    angleQ.setEuler(t2,t1,t3);
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

void rosNode::updateFakeMeasurement(const human_walking_detection::Pose& poseHuman) {
    measurement[0] = poseHuman.x;
    measurement[1] = poseHuman.y;
    // cout<<"position: x = "<<measurement[0]<<", y = "<<measurement[1]<<endl;
}

void rosNode::updateRealMeasurement(const camera_detector::detections& poseHuman) {
    if (poseHuman.detections.size()>0) {
        measurement[0] = cos(thetaRobot) * (poseHuman.detections[0].x) - sin(thetaRobot) * poseHuman.detections[0].y + xRobot;
        measurement[1] = sin(thetaRobot) * (poseHuman.detections[0].x) + cos(thetaRobot) * poseHuman.detections[0].y + yRobot;
    }
}

void rosNode::processMap() {
/// [plot map]
// create markers for visualization
    markerA.ns = "wall";
    markerA.header.frame_id = "/semanticMap";
    markerA.action = visualization_msgs::Marker::ADD;
    semantic_map = n.advertise<visualization_msgs::MarkerArray>("/HWD/semanticMap",3);
    dynamic_map = n.advertise<visualization_msgs::MarkerArray>("/HWD/dynamicMap",3);
// obtain already published map information from yaml file
    XmlRpc::XmlRpcValue walls,AoI;
    n.getParam("/human_walking_detection/walls",walls);
    n.getParam("/human_walking_detection/AoI",AoI);
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

void rosNode::publishTube(human_walking_detection::tubes tube, human_walking_detection::tubesH tubesH) {
    tubeTop.publish(tube);
    tubeHTop.publish(tubesH);
}

void rosNode::publishHypotheses(human_walking_detection::hypotheses hypotheses) {
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
    markerSpeed.header.frame_id = "/semanticMap";
    humanSpeed.publish(markerSpeed);
}

void rosNode::visualizeMeasuredHuman() {
    markerA.ns = "measuredHuman";
    createLine(1,measurement[0],measurement[1],0.0,measurement[0],measurement[1],1.8,0.5,0.5,0.5,0.3,1.0,markerA);
    // createLine(1,4.5,8.4,0.0,4.5,8.4,1.8,0.5,0.5,0.5,0.3,markerA);
    measuredHuman.publish (deleteAllMarker);
    measuredHuman.publish (markerA);
}

void rosNode::visualizeRobot() {
    // markerA.ns = "virtualRobot";
    // // createLine(1,measurement[0],measurement[1],0.0,measurement[0],measurement[1],1.8,0.5,0.5,0.5,0.3,markerA);
    // createLine(1,0.0,1.4,0.0,0.0,1.4,1.8,0.5,0.5,0.5,0.3,1.0,markerA);
    // virtualRobot.publish (deleteAllMarker);
    // virtualRobot.publish (markerA);
}

void rosNode::setStaticMap(visualization_msgs::MarkerArray staticMap) {
    markerArrayStatic = staticMap;
}

void rosNode::setDynamicMap(visualization_msgs::MarkerArray dynamicMap) {
    markerArrayDynamic = dynamicMap;
}