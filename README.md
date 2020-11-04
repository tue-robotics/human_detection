# A probabilistic model for real-time semantic prediction of human's motion intentions
This package adds human estimation functionalities based on a published human position.

## Video - Human Intention Prediction
In this video the performance of the algorithm is visualised. The pictures of the algorithm are shown alongside with the OpenPose detections. The walls are visualised in red, the human in a blue cilinder, the velocity of the human in a blue velocity vector, the local areas to which the detected human could walk in green, the edges of the walking areas in blue, the collision area in red and the robot in a green box. Note that the transparency of the local areas of interest and edges of the walking ares, are correlated to the probability the detected human moves towards that area. Moreover, the uncertainty is shown in purple left to the pictures taken by the robot. Higher means the robot is less sure about the direction the human moves towards, typically seen when the human is changing direction between two areas of interest.
![Human Intention Prediction](figures/performance.gif?raw=true "Human Intention Prediction")


## Quick Start
Publish a human pose in 2D (x and y) at a rostopic /Human/pose and run in a terminal:
```
roslaunch human_walking_detection human_walking_detection.launch 
```
Visualisations could be extracted by using rviz.

## Functions in vectorFieldMap class explained
 linesToTube
 Creates a geometric based vector field between two or more lines including gradient constraints at the end of these lines
 
 findPointInTube
 Returns the fields a detected human in x and y, is currently in.
 
 calcGradient
 Returns the gradient of a field at a position x, y.
 
 addObject
 Adds fields around and towards an object.
 
 createGraph
 Creates graph, representing the connected fields
 
 plotTube
 Visualise a field in RVIZ
 
 walkConstant
 Returns the x,y position at a distance ds perpendicular to a streamlines of a field
 
 
