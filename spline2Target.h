//#include "spline.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifndef SPLINE_H
#define SPLINE_H

typedef struct motionprimitive
{
    int id;
    bool black; //0 not visited, 1 visited
    int motherpointer; //

    double v;
    double omega1;
    double omega2;

    int N; //1,2,3

    double t1;
    double t2;
    double t3;

    double xstart;
    double ystart;
    double thetastart;

    double xend;
    double yend;
    double thetaend;

    motionprimitive()
    {
    }
    motionprimitive(int id_, int N_, int motherpointer_, double v_, double omega1_, double omega2_, double t1_, double t2_, double t3_)
    {
        id = id_;
        black = false;
        motherpointer = motherpointer_;

        N = N_;

        v = v_;
        omega1 = omega1_;
        omega2 = omega2_;

        t1 = t1_;
        t2 = t2_;
        t3 = t3_;
    }

    void print()
    {
        printf("motionprimitive a %d %d %d | %f %f %f %f %f %f\n", id, N, motherpointer, xstart, ystart, thetastart, xend, yend, thetaend);
        printf("motionprimitive b %f %f %f \n", v, omega1, omega2);
        printf("motionprimitive c %f %f %f\n", t1, t2, t3);
        printf("\n");
    }

    void setstartend(double xstart_, double ystart_, double thetastart_, double xend_, double yend_, double thetaend_)
    {
        xstart = xstart_;
        ystart = ystart_;
        thetastart = thetastart_;

        xend = xend_;
        yend = yend_;
        thetaend = thetaend_;
    }
}motionprimitive;


//external
//global planning from (xstart, ystart, thetastart) -> (xtarget, ytarget, thetatarget)
//robot with width robot_w and length robot_h, in meters
//radar data can be directly input by pointcloud in local coordinate (centering robot), point cloud = {{x0, y0}, {x1, y1}, {x2, y2}, ..., {xn, yn}}
//global map can also be input by (map, mapresotluion, originx, originy)
std::vector<motionprimitive> SearchPath(double xstart, double ystart, double thetastart, double xtarget, double ytarget, double thetatarget,
                                        double robot_w, double robot_h,
                                        std::vector<std::vector<float>> pointcloud, cv::Mat map, float mapresolution, float originx, float originy);

//external
//generate trajectory of each motion premitive
//forward1 is one line/circle with no regression
//forward2 is two circles tangent to each other, with regression2
//forward3 is two lines and one circle which tangent to the two lines, with regression3
void forward1(double xstart, double ystart, double thetastart, double v, double omega, double t1, std::vector<std::vector<double>> &traj);
void forward2(double xstart, double ystart, double thetastart, double v, double omega1, double omega2, double time[2], std::vector<std::vector<double>> &traj);
void forward3(double xstart, double ystart, double thetastart, double v, double omega, double time[3], std::vector<std::vector<double>> &traj);

//internel
//search in local coordinate
std::vector<motionprimitive> SearchPathLocalCoordinate(double xstart, double ystart, double thetastart, double xtarget, double ytarget, double thetatarget,
                                        double robot_w, double robot_h,
                                        std::vector<std::vector<float>> pointcloud, cv::Mat map, float mapresolution, float originx, float originy);

bool regression2(double xtarget, double ytarget, double thetatarget, double v, double omega1, double &omega2, double time[2]);
bool regression3(double xtarget, double ytarget, double thetatarget, double v, double omega, double time[3]);

bool _check_collision_robot(double w, double h, double x, double y, double theta, std::vector<std::vector<float>> pointcloud);
bool _check_collision_map(cv::Mat map, float mapresolution, float originx, float originy, float robot_w, float robot_h, std::vector<std::vector<double>> traj);

void Transform2D(double xp, double yp, double &x, double &y, double xstart, double ystart, double thetastart);

#endif