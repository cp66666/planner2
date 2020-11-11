#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>    // std::reverse
#include <Eigen/Dense>
using namespace Eigen;

#define SPLINE_DT 0.5

#include "spline2Target.h"

unsigned char* gTrajMap = NULL;

void _roundtheta(double &theta)
{
  if (theta > CV_PI)
  {
    theta -= 2 * CV_PI;
  }
  else if (theta <= -CV_PI)
  {
    theta += 2 * CV_PI;
  }
}

double clockdiff(double angle0, double angle1, double dir)
{
    double theta;
    if (dir > 0)
    {
        theta = angle1 - angle0;
    }
    else
    {
        theta = angle0 - angle1;
    }
    if (theta < 0)
    {
        theta += CV_PI*2.0;
    }
    if (theta > CV_PI*2.0)
    {
        theta -= CV_PI*2.0;
    }
    return theta;
}

//local to global
void Transform2D(double xlocal, double ylocal, double &x, double &y, double xstart, double ystart, double thetastart)
{
    x = xlocal*cos(thetastart) - ylocal*sin(thetastart) + xstart;
    y = ylocal*cos(thetastart) + xlocal*sin(thetastart) + ystart;
}

bool _check_collision_robot(double w, double h, double x, double y, double theta, std::vector<std::vector<float>> pointcloud)
{
    bool bcollision = false;
    std::vector<std::vector<float>> pointcloudcurrent;
    for (int i = 0; i < pointcloud.size(); i++)
    {
        std::vector<float> pt = pointcloud[i];
        float dx = pt[0] - x;
        float dy = pt[1] - y;
        float x_ = -cos(theta - CV_PI/2.0)*dx - sin(theta - CV_PI/2.0)*dy;
        float y_ = -sin(theta - CV_PI/2.0)*dx + cos(theta - CV_PI/2.0)*dy;

        if (x_ >= -0.5*w && x_ <= 0.5*w)
        {
            if (y_ >= -0.5*h && y_ <= 0.5*h)
            {
                bcollision = true;
            }
        }
    }
    return bcollision;
}

bool _check_collision_ptcloud(double w, double h, std::vector<std::vector<double>> traj, std::vector<std::vector<float>> pointcloud)
{
    bool bcollision = false;
    for (int j = 0; j < traj.size(); j++)
    {
        double x = traj[j][0];
        double y = traj[j][1];
        double theta = traj[j][2];


        bcollision = bcollision | _check_collision_robot(w, h, x, y, theta, pointcloud);
    }
    return bcollision;
}

bool _check_collision_map(cv::Mat map, float mapresolution, float originx, float originy, float robot_w, float robot_h, std::vector<std::vector<double>> traj)
{
    bool bcollision = false;

    int wmap = map.cols;
    int hmap = map.rows;
    float ms = mapresolution;

    if (gTrajMap == NULL)
    {
        gTrajMap = new unsigned char[wmap*hmap];
    }
    memset(gTrajMap, 255, wmap*hmap);
    cv::Mat white_board(cv::Size(wmap, hmap), CV_8U, (void*)gTrajMap, cv::Mat::AUTO_STEP);

    for (int l = 0; l < traj.size(); l++)
    {
        float x = traj[l][0];
        float y = traj[l][1];
        float theta = traj[l][2];

        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f((x - originx)/ms,  (y - originy)/ms), cv::Size2f(robot_w/mapresolution, robot_h/mapresolution), (theta - CV_PI/2.0)*180/CV_PI);
        cv::Point2f vertices_[4];
        rRect.points(vertices_);

        cv::Point vertices[4];
        for (int k = 0; k < 4; k++)
        {
            vertices[k] = cv::Point(vertices_[k]);
        }
        cv::fillConvexPoly(white_board, vertices, 4, cv::Scalar(0));
    }

    for (int j = 0; j < hmap; j++)
    {
        if (bcollision)
        {
            break;
        }
        for (int i = 0; i < wmap; i++)
        {
            unsigned char ob = map.data[j*wmap + i];
            unsigned char tr = white_board.data[j*wmap + i];

            if ((ob|tr) == 0)
            {
                bcollision = true;
                break;
            }
        }
    }

    printf("!!! bcollision %d\n", bcollision);
    return bcollision;
}

bool regression2(double xtarget, double ytarget, double thetatarget, double v, double omega1, double &omega2, double time[2])
{
    double R1 = v/omega1;
    double a1 = 2*(xtarget + R1)*cos(thetatarget) - 2*R1 + 2*ytarget*sin(thetatarget);
    double a2 = -2*(xtarget + R1)*cos(thetatarget) - 2*R1 - 2*ytarget*sin(thetatarget);
    double b = R1*R1 - ytarget*ytarget - (xtarget + R1)*(xtarget + R1);

    double R21 = b/a1;
    double R22 = b/a2;
    double R2;

    double x0, y0;
    double theta1;
    double theta2;
    double t1, t2;

    if (std::abs(R21) <= std::abs(R22))
    {
        R2 = R21;

        x0 = xtarget + cos(thetatarget)*R21;
        y0 = ytarget + sin(thetatarget)*R21;

        omega2 = -v/R21;
        double theta_ = atan2(y0, R1 + x0);
        double angle0, angle1, angle1p, angle2;
        if (R1 > 0)
        {
            angle0 = 0.0;
            angle1 = theta_;
            angle2 = thetatarget;

            theta1 = clockdiff(angle0, angle1, omega1);
            theta2 = clockdiff(angle1, angle2, omega2);

            //printf("a0 %f %f %f %f | %f %f\n", x0, y0, angle1*180/CV_PI, angle2*180/CV_PI, theta1*180/CV_PI, theta2*180/CV_PI);
        }
        else
        {
            angle0 = CV_PI;
            angle1 = theta_;
            angle2 = thetatarget;

            theta1 = clockdiff(angle0, angle1, omega1);
            theta2 = clockdiff(CV_PI + angle1, angle2, omega2);

            //printf("a1 %f %f | %f %f %f | %f %f\n", x0, y0, angle0*180/CV_PI, angle1*180/CV_PI, angle2*180/CV_PI, theta1*180/CV_PI, theta2*180/CV_PI);
        }
    }
    else
    {
        R2 = R22;
        x0 = xtarget + cos(thetatarget)*R22;
        y0 = ytarget + sin(thetatarget)*R22;
        omega2 = -v/R22;
        double theta_ = atan2(y0, R1 + x0);

        double angle0, angle1, angle1p, angle2;
        if (R1 > 0)
        {
            angle0 = 0.0;
            angle1 = theta_;
            angle2 = thetatarget;

            theta1 = clockdiff(angle0, angle1, omega1);
            theta2 = clockdiff(angle1, angle2, omega2);

            // printf("%f %f %f | %f %f\n", R1, R21, R22, omega1, omega2);
           // printf("b0 %f %f | %f %f | %f %f\n", x0, y0, angle1*180/CV_PI, angle2*180/CV_PI, theta1*180/CV_PI, theta2*180/CV_PI);
        }
        else
        {
            angle0 = CV_PI;
            angle1 = theta_;
            angle2 = thetatarget;

            theta1 = clockdiff(angle0, angle1, omega1);
            theta2 = clockdiff(CV_PI + angle1, angle2, omega2);

          //  printf("b1 %f %f | %f %f %f | %f %f\n", x0, y0, angle0*180/CV_PI, angle1*180/CV_PI, angle2*180/CV_PI, theta1*180/CV_PI, theta2*180/CV_PI);
        }
    }


    t1 = theta1/std::abs(omega1);
    t2 = t1 + theta2/std::abs(omega2);

    time[0] = t1;
    time[1] = t2;

    double dis1 = sqrt((x0+R1)*(x0+R1) + y0*y0) - std::abs(R1) - std::abs(R2);
    //printf("!!%f %f  %f %f  %f\n", t1, t2, omega1, omega2, dis1);

    if (t1 > 0 && t2 > t1 && std::abs(theta1) <= CV_PI && std::abs(theta2) <= CV_PI && dis1 < 0.0001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void forward1(double xstart, double ystart, double thetastart, double v, double omega, double t1, std::vector<std::vector<double>> &traj)
{
    traj.clear();

    double t = 0.0;
    double dt = SPLINE_DT;
    while (t < t1)
    {
        double R = v/omega;

        double xp;
        double yp;
        double theta;
        double x, y;

        if (omega != 0.0)
        {
            xp = R*(cos(omega*t) - 1);
            yp = R*sin(omega*t);

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            theta = omega*t + thetastart;
            _roundtheta(theta);
        }
        else
        {
            xp = 0.0;
            yp = v*t;

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            theta = thetastart;
        }


        std::vector<double> p3 = {x, y, theta, v, omega, t};
        traj.push_back(p3);

        t += dt;
    }
}

void forward2(double xstart, double ystart, double thetastart, double v, double omega1, double omega2, double time[2], std::vector<std::vector<double>> &traj)
{
    traj.clear();

    double t1 = time[0];
    double t2 = time[1];

    double R1 = v/omega1;
    double R2 = v/omega2;

    double dt = SPLINE_DT;

    double xp, yp;
    double x, y, theta;
    double t = 0.0;
    //omega2 = -omega2;
    while (t < t2)
    {
        if (t < t1)
        {
            xp = R1*(cos(omega1*t) - 1);
            yp = R1*sin(omega1*t);

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            theta = omega1*t + thetastart;
            _roundtheta(theta);

            std::vector<double> p3 = {x, y, theta, v, omega1, t};
            traj.push_back(p3);
        }
        else
        {
            double dxp = R2*(cos(omega2*(t - t1)) - 1);
            double dyp = R2*sin(omega2*(t - t1));

            Transform2D(dxp, dyp, xp, yp, R1*(cos(omega1*t1) - 1), R1*sin(omega1*t1), omega1*t1);
            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            theta = omega1*t1 + omega2*(t - t1);
            _roundtheta(theta);
            theta = theta + thetastart;
            _roundtheta(theta);

            std::vector<double> p3 = {x, y, theta, v, omega2, t};
            traj.push_back(p3);
        }
        t += dt;
    }
}

void forward3(double xstart, double ystart, double thetastart, double v, double omega, double time[3], std::vector<std::vector<double>> &traj)
{
    traj.clear();

    double t1 = time[0];
    double t2 = time[1];
    double t3 = time[2];

    double thetatarget = omega*(t2 - t1);

    double x1p = 0.0;
    double y1p = v*t1;

    double x2p = x1p + v*(cos(thetatarget)-1.0)/omega;
    double y2p = y1p + v*sin(thetatarget)/omega;

    double x3p = x2p - v*sin(thetatarget)*(t3 - t2);
    double y3p = y2p + v*cos(thetatarget)*(t3 - t2);

    double x1, y1, x2, y2, x3, y3;

    Transform2D(x1p, y1p, x1, y1, xstart, ystart, thetastart);
    Transform2D(x2p, y2p, x2, y2, xstart, ystart, thetastart);
    Transform2D(x3p, y3p, x3, y3, xstart, ystart, thetastart);

    double t = 0.0;
    double dt = SPLINE_DT;
    double theta, x, y, xp, yp;
    while(t <= t3)
    {
        if (t < t1)
        {
            xp = 0.0;
            yp = v*t;
            theta = thetastart;

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            std::vector<double> p3 = {x, y, theta, v, 0.0, t};
            traj.push_back(p3);
        }
        else if (t < t2)
        {
            xp = x1p + v*(cos(omega*(t - t1))-1.0)/omega;
            yp = y1p + v*sin(omega*(t - t1))/omega;
            theta = thetastart + (t - t1)*omega;
            _roundtheta(theta);

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            std::vector<double> p3 = {x, y, theta, v, omega, t};
            traj.push_back(p3);
        }
        else
        {
            xp = x2p - v*sin(thetatarget)*(t - t2);
            yp = y2p + v*cos(thetatarget)*(t - t2);
            theta = thetastart + thetatarget;
            _roundtheta(theta);

            Transform2D(xp, yp, x, y, xstart, ystart, thetastart);

            std::vector<double> p3 = {x, y, theta, v, 0.0, t};
            traj.push_back(p3);
        }
        t += dt;
    }
}

bool regression3(double xtarget, double ytarget, double thetatarget, double v, double omega, double time[3])
{

    Eigen::Matrix2d A;
    A << v, v*cos(thetatarget), 0.0, -v*sin(thetatarget);

    Eigen::Vector2d b(ytarget - v*sin(thetatarget)/omega, xtarget - v*(cos(thetatarget)-1.0)/omega);
    Eigen::Vector2d C = A.colPivHouseholderQr().solve(b);

    double u1 = C(0);
    double u2 = C(1);

  //  printf("!! %f %f\n", v, omega);

    double t1 = u1;
    double t2 = t1 + thetatarget/omega;
    double t3 = t2 + u2;

    time[0] = t1;
    time[1] = t2;
    time[2] = t3;

   // printf("!! u1, u2 = %f %f\n", u1, u2);

    if (t1 > 0 && t2 > t1 && t3 > t2)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::vector<double>> Local2Global(std::vector<std::vector<double>> local_traj, double xstart, double ystart, double thetastart)
{
    std::vector<std::vector<double>> global_traj;
    //local to global
    for (int i = 0; i < local_traj.size(); i++)
    {
        double x0 = local_traj[i][0];
        double y0 = local_traj[i][1];
        double theta0 = local_traj[i][2];

        double x, y, theta;

        //
        Transform2D(x0, y0, x, y, xstart, ystart, thetastart);
        theta = theta0 + thetastart;
        _roundtheta(theta);

        std::vector<double> gpt = {x, y, theta};

        global_traj.push_back(gpt);
    }
    return global_traj;
}

std::vector<motionprimitive> Local2Global(std::vector<motionprimitive> local_traj, double xstart, double ystart, double thetastart)
{
    std::vector<motionprimitive> global_traj;
    //local to global
    for (int i = 0; i < local_traj.size(); i++)
    {
        motionprimitive m = local_traj[i];

        double x, y, theta;

        //
        Transform2D(m.xstart, m.ystart, x, y, xstart, ystart, thetastart);
        theta = m.thetastart + thetastart;
        _roundtheta(theta);
        m.xstart = x;
        m.ystart = y;
        m.thetastart = theta;

        //
        Transform2D(m.xend, m.yend, x, y, xstart, ystart, thetastart);
        theta = m.thetaend + thetastart;
        _roundtheta(theta);
        m.xend = x;
        m.yend = y;
        m.thetaend = theta;

        global_traj.push_back(m);
    }
    return global_traj;
}


std::vector<motionprimitive> SearchPath(double xstart, double ystart, double thetastart, double xtarget, double ytarget, double thetatarget,
                                        double robot_w, double robot_h,
                                        std::vector<std::vector<float>> pointcloud, cv::Mat map, float mapresolution, float originx, float originy)
{
    double dtheta = thetatarget - thetastart;
    _roundtheta(dtheta);
    double x_, y_;
    Transform2D(xtarget - xstart, ytarget - ystart, x_, y_, 0.0, 0.0, -thetastart);

    std::vector<motionprimitive> local_traj = SearchPathLocalCoordinate(xstart, ystart, thetastart, x_, y_, dtheta, robot_w, robot_h, pointcloud, map, mapresolution, originx, originy);
    std::vector<motionprimitive> global_traj = Local2Global(local_traj, xstart, ystart, thetastart);

    return global_traj;
}

std::vector<motionprimitive> SearchPathLocalCoordinate(double xstart, double ystart, double thetastart, double xtarget, double ytarget, double thetatarget,
                                        double robot_w, double robot_h,
                                        std::vector<std::vector<float>> pointcloud, cv::Mat map, float mapresolution, float originx, float originy)
{
    double w = robot_w;
    double h = robot_h;

    std::vector<motionprimitive> openList;
    std::vector<motionprimitive> closeList;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double v = 0.1;
    double omegafwd1[9] = {0.0, -0.05, 0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2};
    double omegafwd2[8] = {-0.05, 0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2};

    double T = 0.5/v;
    int motherpointer = -1;
    int nodeid = 0;

    //start is an motionprivitive with no motion
    motionprimitive mp(0, 0, motherpointer, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    printf("generate node %d\n", nodeid);
    mp.setstartend(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    openList.push_back(mp);
    nodeid++;

    int searchover = 0;
    std::vector<std::vector<double>> traj;

    for (int i = 0; i < 1000; i++)
    {
        printf("\n\n");
        //choose one from openlist as start point
        bool allvisited = true;
        int regressstartid = -1;

        for (int j = 0; j < openList.size(); j++)
        {
            motionprimitive mp = openList[j];
            if (!mp.black)
            {
                printf("choose %d\n", mp.id);
                x = mp.xend;
                y = mp.yend;
                theta = mp.thetaend;
                regressstartid = mp.id;
                closeList.push_back(mp);
                openList[j].black = true;
                allvisited = false;
            }
        }

        //regression see whether search is over
        if (!allvisited)
        {
            int minsize2 = INT_MAX;
            int minsize3 = INT_MAX;
            motionprimitive mpmin2;
            motionprimitive mpmin3;

            double dtheta = thetatarget - theta;
            _roundtheta(dtheta);
            double x_, y_;
            Transform2D(xtarget - x, ytarget - y, x_, y_, 0.0, 0.0, -theta);


            for (int j = 0; j < 8; j++)
            {
                double vj = v;
                double omegaj = omegafwd2[j];

                double t2[2];
                double t3[3];

                double omega2;
               // printf("3 %f %f %f %f %f %f\n", x_, y_, dtheta, xtarget, ytarget, thetatarget);
                bool breg3 = regression3(x_, y_, thetatarget, v, omegaj, t3);
                if (breg3)
                {
                    forward3(x, y, theta, v, omegaj, t3, traj);

                    std::vector<std::vector<double>> global_traj = Local2Global(traj, xstart, ystart, thetastart);
                    bool colli = _check_collision_ptcloud(w, h, traj, pointcloud);
                    colli |= _check_collision_map(map, mapresolution, originx, originy, robot_w, robot_h, global_traj);

                    if (!colli)
                    {
                        int size = traj.size();
                        if (size < minsize3)
                        {
                            minsize3 = size;
                            motionprimitive mp(nodeid, 3, regressstartid, vj, omegaj, omega2, t3[0], t3[1], t3[2]);
                            mp.setstartend(x, y, theta, xtarget, ytarget, thetatarget);
                            mpmin3 = mp;
                            searchover = 1;
                            printf("!! found reg3\n");

                        }
                        break;
                    }
                }

                bool breg2 = regression2(x_, y_, dtheta, v, omegaj, omega2, t2);
                if (breg2)
                {
                    forward2(x, y, theta, v, omegaj, omega2, t2, traj);

                    std::vector<std::vector<double>>  global_traj = Local2Global(traj, xstart, ystart, thetastart);
                    bool colli = _check_collision_ptcloud(w, h, traj, pointcloud);
                    colli |= _check_collision_map(map, mapresolution, originx, originy, robot_w, robot_h, global_traj);

                    if (!colli)
                    {
                        int size = traj.size();
                        if (size < minsize2)
                        {
                            minsize2 = size;
                            motionprimitive mp(nodeid, 2, regressstartid, vj, omegaj, omega2, t2[0], t2[1], 0.0);
                            mp.setstartend(x, y, theta, xtarget, ytarget, thetatarget);
                            mpmin2 = mp;
                            searchover = 2;
                            printf("!! found reg2 %d\n", regressstartid);
                        }
                        break;
                    }
                }
            }
            if (searchover != 0)
            {
                printf("!! search over %d\n", searchover);
                if (searchover == 1)
                {
                    closeList.push_back(mpmin3);
                }
                if (searchover == 2)
                {
                    closeList.push_back(mpmin2);
                }
                break;
            }
        }
        //if openlist is all visited, make new openlist
        else
        {
            motionprimitive mp0 = openList[0];
            motherpointer = mp0.id;

            printf("allvisited, generate next level from node %d\n", motherpointer);

            for (int j = 0; j < 9; j++)
            {
                double vj = v;
                double omegaj = omegafwd1[j];
                forward1(mp0.xend, mp0.yend, mp0.thetaend, vj, omegaj, T, traj);

                std::vector<std::vector<double>> global_traj = Local2Global(traj, xstart, ystart, thetastart);
                bool colli = _check_collision_ptcloud(w, h, traj, pointcloud);
                colli |= _check_collision_map(map, mapresolution, originx, originy, robot_w, robot_h, global_traj);

                if (!colli)
                {
                    std::vector<double> end = traj[traj.size() - 1];
                    //printf("end = %f %f %f\n", end[0], end[1], end[2]);
                    printf("generate node %d\n", nodeid);
                    motionprimitive mp(nodeid, 1, motherpointer, vj, omegaj, 0.0, T, 0.0, 0.0);
                    mp.setstartend(mp0.xend, mp0.yend, mp0.thetaend, end[0], end[1], end[2]);
                    openList.push_back(mp);
                    nodeid++;
                }
            }
            openList.erase(openList.begin());
        }
    }

    std::vector<motionprimitive> output;
    motionprimitive mpo = closeList[closeList.size() - 1];
    output.push_back(mpo);
    while(mpo.motherpointer != -1)
    {
      //  printf("%d %d\n", mpo.id, mpo.motherpointer);
        int l = mpo.motherpointer;
        int il = -1;
        for (int i = 0; i < closeList.size(); i++)
        {
            if (closeList[i].id == l)
            {
                il = i;
            }
        }
        mpo = closeList[il];
        output.push_back(mpo);
    }

    std::reverse(output.begin(), output.end());
    return output;
}