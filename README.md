# Planner2, the Test Repo for New Algorithm 

### Purpose of planner2:
* test new API in simulation and runtime
* output awesome demos 
* output data for paper

### planner2 -> planner1
planner2 only release functions or API to planner1 by copy paste, under architect of planner1

### global planning
please refer to the only API for global planning
and the example code
```
//external
//global planning from (xstart, ystart, thetastart) -> (xtarget, ytarget, thetatarget)
//robot with width robot_w and length robot_h, in meters
//radar data can be directly input by pointcloud in local coordinate (centering robot), point cloud = {{x0, y0}, {x1, y1}, {x2, y2}, ..., {xn, yn}}
//global map can also be input by (map, mapresotluion, originx, originy)
std::vector<motionprimitive> SearchPath(double xstart, double ystart, double thetastart, double xtarget, double ytarget, double thetatarget,
                                        double robot_w, double robot_h,
                                        std::vector<std::vector<float>> pointcloud, cv::Mat map, float mapresolution, float originx, float originy);
```

### collision avoidance
the following functions will be overload in planner2
whenever one already has the velocity calculated by local planning and PID
```
v
omega
```
one need to update omega by collision avoidance

```
void getMaxCollisionAvoidanceOmega(double halfH, double halfW, std::vector<std::vector<float>> pointcloud, double v, double &omegaleftbound, double &omegarightbound);
void getMinCollisionAvoidanceOmega(double halfH, double halfW, double forwarddis, double v, std::vector<std::vector<float>> pointcloud, int &stop, double &omega);
```

MinCollisionAvoidanceOmega is the min omega to avoid head collision under v deteremined by local planning
MaxCollisionAvoidanceOmega is the max omega to avoid butt collision under v determined by local planning

If omega calculated fall out of [minomega, maxomega], it is set to be the corresponding boundary value, either minomega or maxomega
# planner2
