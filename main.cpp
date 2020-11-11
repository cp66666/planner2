#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <thread>

#include <random>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include "spline2Target.h"
#include "path2D.h"
#include "RobotControl.h"
#include "CollisionAvoidance.h"

#include "dds_participant.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"

#include "Callbacks.h"
#include "MessageParser.h"
cmdrDDS::DdsParticipant participant_ca;
cmdrDDS::DdsParticipant participant_carto;
cmdrDDS::DdsParticipant participant_robot;
cmdrDDS::DdsParticipant participant_planner;
cmdrDDS::DdsParticipant participant_ca_lidar;

///
cmdrDDS::DdsSubscriber<commander_robot_msg::PoseStamped, commander_robot_msg::PoseStampedPubSubType> subscriber_pose;
cmdrDDS::DdsSubscriber<commander_robot_msg::LaserScan, commander_robot_msg::LaserScanPubSubType> subscriber_laser1;
cmdrDDS::DdsSubscriber<commander_robot_msg::LaserScan, commander_robot_msg::LaserScanPubSubType> subscriber_laser2;
cmdrDDS::DdsSubscriber<collisionavoidance_msg, collisionavoidance_msgPubSubType> subscriber_realsense1;

using namespace planner2;
int main()
{
    //begin subscribe
    subscriber_pose.Create(participant_carto, "curr_pose", 1, poseCallback);
    subscriber_realsense1.Create(participant_ca, "collisionavoidance_msg", 1, realsense1_callback);
    subscriber_laser1.Create(participant_carto, "scan_1", 1, laser1_callback);
    subscriber_laser2.Create(participant_carto, "scan_2", 1, laser2_callback);

    cmdrDDS::DdsPublisher<commander_msg::StdMsg, commander_msg::StdMsgPubSubType> robotcomm_publisher;
    commander_msg::StdMsg planner_robotcomm_msg;
    robotcomm_publisher.Create(participant_robot, "planner_msg", 1);// 0905

    //map info
    float mapresolution = 0.01;
    float originx = -12.0;
    float originy = -6.0;
    float robot_w = 1.0;
    float robot_h = 0.5;

    //map and rendering image
    cv::Mat m_test =  cv::Mat(1600, 1600, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat m_map = cv::Mat(1600, 1600, CV_8UC1, cv::Scalar(255));

    //generate point cloud in local coordinate
    //and draw them in map
    std::vector<std::vector<float>> pointcloud;
    std::vector<std::vector<float>> pointcloud_empty;
    for (int i = 0; i < 20; i++)
    {
        std::vector<float> pt = {0.0, 0.0};
        pt[1] = -0.5 + 0.2*i;
        pt[0] = -1.5;
        pointcloud.push_back(pt);
        cv::circle(m_test, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0, 0, 0), -1);
        cv::circle(m_map, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0), -1);

        pt[1] = 3.9;
        pt[0] = -4.3 + 0.03*i;
        pointcloud.push_back(pt);
        cv::circle(m_test, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0, 0, 0), -1);
        cv::circle(m_map, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0), -1);

        pt[1] = -2.0;
        pt[0] = 0.1 + 0.03*i;
        pointcloud.push_back(pt);
        cv::circle(m_test, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0), -1);
        cv::circle(m_map, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 3, cv::Scalar(0), -1);
    }

    //suddenly appear some obstacle, don't draw them in map
    for (int i = 0; i < 10; i++)
    {
        std::vector<float> pt = {0.0, 0.0};
        pt[1] = 5.3 + 0.04*i;
        pt[0] = -1.2 - 0.1*i;
        pointcloud.push_back(pt);
        cv::circle(m_test, cv::Point((int)(1200 + 100*pt[0]), (int)(600 + 100*pt[1])), 1, cv::Scalar(255, 0, 0), -1);
    }


    //start point and target for test
    std::vector<std::vector<double>> traj;
    double xs = 0.5;
    double ys = 0.5;
    double thetas = CV_PI/6.0;
    double xt = -8.0;
    double yt = 6.0;
    double thetat = CV_PI/2.0;
    cv::circle(m_test, cv::Point((int)(1200 + 100*(xs)), (int)(600 + 100*(ys))), 10, cv::Scalar(0, 0, 255), 5);
    cv::circle(m_test, cv::Point((int)(1200 + 100*(xt)), (int)(600 + 100*(yt))), 10, cv::Scalar(0, 0, 255), 5);

    //the only API for global planning
    //NOTE point cloud is in local coordinate (centering robot), because we draw all point cloud in the map, we use a empty point cloud here
    std::vector<motionprimitive> mps = SearchPath(xs, ys, thetas, xt, yt, thetat, 0.5, 1.0, pointcloud_empty, m_map, 0.01, -12.0, -6.0);

    //now we have a vector of motionprimitives
    //using calc_global_paths to calculate the path
    //&lat_qp can be NULL
    //go around obstacle by assign d_halfT none zero
    double velocity = 0.5;
    double d_halfT = 0.1;
    cos_poly33 lat_qp =  cos_poly33(0, 0.1, 0, 1.0, d_halfT, 0.0);
    Spline2D path = calc_global_paths(mps, &lat_qp, velocity);

    //rendering
    for (int i = 0; i < path.m_s.size() - 1; i++)
    {
        double s0 = path.m_s[i];
        double x, y, yaw;
        path.getPos(s0, x, y, yaw);

        cv::circle(m_test, cv::Point((int)(1200 + 100*x), (int)(600 + 100*y)), 3, cv::Scalar(0), 1);
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(1200 + x*100,  600 + y*100), cv::Size2f(1.0*100, 0.5*100), yaw*180/CV_PI);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int j = 0; j < 4; j++)
        {
            cv::line(m_test, vertices[j], vertices[(j+1)%4], cv::Scalar(225, 225, 225), 1);
        }
    }

    //prepare simulation
    stRobotPG simlatePG(xs, ys, thetas + CV_PI/2.0);
    RobotControl *robotcontrol = new RobotControl();
    robotcontrol->Start();

    CRobotSimulator *robotSimulator = new CRobotSimulator(robotcontrol, simlatePG, 10, 0.5);
    robotSimulator->Start();
    stRobotPG currPG;

    double starget, dtarget;
    calc_frenet_pos(xt, yt, path, dtarget, starget);

    double vlast, omegalast;

    //simulation
    while (true)
    {
        cv::Mat m_test2 = cv::Mat(1600, 1600, CV_8UC3, cv::Scalar(255));
        memcpy(m_test2.data, m_test.data, 1600*1600*3);

        //get current pose
        currPG = robotSimulator->GetRealPG();
        printf("aaa %f %f\n", currPG.x, currPG.y);

        //convert point cloud to local coordinate
        std::vector<std::vector<float>> pointcloudcurrent;
        for (int i = 0; i < pointcloud.size(); i++)
        {
            std::vector<float> pt = pointcloud[i];
            float dx = pt[0] - currPG.x;
            float dy = pt[1] - currPG.y;
            float x_ = -cos(currPG.theta - CV_PI/2.0)*dx - sin(currPG.theta - CV_PI/2.0)*dy;
            float y_ = -sin(currPG.theta - CV_PI/2.0)*dx + cos(currPG.theta - CV_PI/2.0)*dy;
            std::vector<float> ptl = {x_, y_};
            pointcloudcurrent.push_back(ptl);
        }

        //PID
        double s, d;
        calc_frenet_pos(currPG.x,  currPG.y, path, d, s);


        double vi, omegai;
        double v, omega;

        printf("bbb 0 %f %f\n", s, d);

        path.getVelocity(s, vi, omegai);

        vlast = v;
        omegalast = omega;

        printf("bbb 1 %f %f\n", vi, omegai);

        double yaw = path.calc_yaw(s);
        float dtheta = yaw - currPG.theta;
        RoundTheta(dtheta);
        PID(vi, omegai, vlast, omegalast, dtheta, s, d, starget, v, omega);
        printf("bbb 2 %f %f\n", v, omega);


        //
        //collision avoidance

        //to avoid butt collision
        double omegaleftbound, omegarightbound;

        getMaxCollisionAvoidanceOmega(0.5 + 0.01, 0.25 + 0.01, pointcloudcurrent, velocity, omegaleftbound, omegarightbound);

        bool testc = false;
        if (omega > 0 && omega > omegarightbound)
        {
            omega = omegarightbound;
        }
        if (omega < 0 && omega < -omegaleftbound)
        {
            omega = -omegaleftbound;
        }

        printf("bbb 3 %f %f\n", v, omega);

        //to avoid head collision
        int stop;
        double omegaca;
        getMinCollisionAvoidanceOmega(0.5 + 0.01, 0.25 + 0.01, 0.5, velocity, pointcloudcurrent, stop, omegaca);

        if (omega*omegaca < 0)
        {
            omega = omegaca;
        }
        else
        {
            if (omega > 0)
            {
                omega = std::max(omega, omegaca);
            }
            else
            {
                omega = std::min(omega, omegaca);
            }
        }

        //send v, omega to robot

        robotcontrol->SetOdometry(v, omega);

        //rendering
        cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(1200 + currPG.x*100,  600 + currPG.y*100), cv::Size2f(1.0*100, 0.5*100), currPG.theta*180/CV_PI);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::line(m_test2, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,0), 1);
        }

        cv::flip(m_test2, m_test2, 0);
        cv::imshow("test", m_test2);
        cv::waitKey(10);

    }
    return 0;
}