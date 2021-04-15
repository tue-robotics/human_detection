## interpolated coordinates (at height of the laser range finder)
#float32 x
#float32 y
#float32 z
## detection probability, radius and length
#float32 p
#float32 r
#float32 d
## coordinates of interpolated lower leg parts (from ankle(0.0) to knee(1.0) @0.0, 0.25, 0.5, 0.75, 1.0)
#float32[] xArray
#float32[] yArray
#float32[] zArray
## coordinates of measured lower leg parts (from ankle(0.0) to knee(1.0) @0.0, 0.25, 0.5, 0.75, 1.0)
#float32[] xValArray
#float32[] yValArray
#float32[] zValArray

## ankle and knee xyz points
#float32 xAnkle
#float32 yAnkle
#float32 zAnkle
#float32 xKnee
#float32 yKnee
#float32 zKnee
## measurement labels
#bool validSize
##bool associated
#bool validHeight
#bool validUniqueness #occluded behind
##int32[] associatedPoints
##float32[] associationProbabilities
##int32[] associatedPointsAll
##float32[] associationProbabilitiesAll
##bool[] associationVisitedAll
##bool associationVisited
#float32 distContinuous
#int32 personID
#bool leftLeg

float32 x
float32 y
float32 z
float32 p
