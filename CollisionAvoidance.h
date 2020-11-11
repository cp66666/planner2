#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "path2D.h"

namespace planner2
{
    void getMaxCollisionAvoidanceOmega(double halfH, double halfW, std::vector<std::vector<float>> pointcloud, double v, double &omegaleftbound, double &omegarightbound);
    void getMinCollisionAvoidanceOmega(double halfH, double halfW, double forwarddis, double v, std::vector<std::vector<float>> pointcloud, int &stop, double &omega);


    void calc_frenet_pos(double x0, double y0, Spline2D csp, double &d, double &s);
    void PID(double vi, double omegai, double vlast, double omegalast, double dtheta, double s, double d, double starget, double &v, double &omega);
}