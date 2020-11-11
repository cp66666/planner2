#include <vector>
#include "CollisionAvoidance.h"
namespace planner2
{
    void PointProjectToLine(std::vector<double> p1, std::vector<double> p2, std::vector<double> Q, std::vector<double> &Proj, double &f)
    {
        double ap[2];
        ap[0] = Q[0] - p1[0];
        ap[1] = Q[1] - p1[1];

        double ab[2];
        ab[0] = p2[0] - p1[0];
        ab[1] = p2[1] - p1[1];

        f = (ap[0]*ab[0] + ap[1]*ab[1])/(ab[0]*ab[0] + ab[1]*ab[1]);
        Proj[0] = p1[0] + f*ab[0];
        Proj[1] = p1[1] + f*ab[1];
    }

    void calc_frenet_pos(double x0, double y0, Spline2D csp, double &d, double &s)
    {
        double pos[2];
        pos[0] = x0;
        pos[1] = y0;

        int minSid = -1;
        double minS = -1.0;
        std::vector<double> posMin = {0.0, 0.0};

        //cv::Mat m_test =  cv::Mat(1200, 1200, CV_8U, cv::Scalar(255));
        //cv::circle(m_test, cv::Point((int)(600 + 400*x0), (int)(200 + 400*y0)), 5, cv::Scalar(0), -1);

         for (int i = 0; i < csp.m_s.size() - 1; i++)
        {
            double s0 = csp.m_s[i];
            std::vector<double> p0 = csp.calc_position(s0);
           // printf("!!%f %f %f\n", s0, p0[0], p0[1]);
            //cv::circle(m_test, cv::Point((int)(600 + 400*p0[0]), (int)(200 + 400*p0[1])), 1, cv::Scalar(55), -1);
        }
        //cv::imshow("test", m_test);
        //cv::waitKey(1000);

    //    //point project to line
    //    for (int i = 0; i < csp.m_s.size() - 1; i++)
    //    {
    //        double s0 = csp.m_s[i];
    //        std::vector<double> p0 = csp.calc_position(s0);
    //        double s1 = csp.m_s[i+1];
    //
    //        //printf("a0000 %d %f %f\n", i, s0, s1);
    //        cv::circle(m_test, cv::Point((int)(600 + 400*p0[0]), (int)(200 + 400*p0[1])), 1, cv::Scalar(55), -1);
    //
    //        std::vector<double> p1 = csp.calc_position(s1);
    //        std::vector<double> Q = {x0, y0};
    //        std::vector<double> Proj = {0.0, 0.0};
    //        double f = -1.0;
    //        PointProjectToLine(p0, p1, Q, Proj, f);
    //
    //        //printf("%d %f %f | %f %f | %f %f | %f %f\n", p0[0], p0[1], p1[0], p1[1], Q[0], Q[1], Proj[0], Proj[1]);
    //        if (f >= 0.0 && f <= 1.0)
    //        {
    //            printf("aaaaaa %d %d | p0 = %f %f %f | p1 = %f %f %f | Q = %f %f %f %f %f\n", i, csp.m_s.size(), p0[0], p0[1], s0, p1[0], p1[1], s1, f,
    //                Q[0], Q[1], Proj[0], Proj[1]);
    //
    //            //cv::circle(m_test, cv::Point((int)(1400 + 1000*p1[0]), (int)(1000*p1[1] - 600)), 1, cv::Scalar(125), -1);
    //            //cv::circle(m_test, cv::Point((int)(1400 + 1000*Q[0]), (int)(1000*Q[1] - 600)), 1, cv::Scalar(125), -1);
    //            //cv::circle(m_test, cv::Point((int)(1400 + 1000*Proj[0]), (int)(1000*Proj[1] - 600)), 1, cv::Scalar(155), -1);
    //
    //            posMin = Proj;
    //            minS = s0 + f*(s1 - s0);
    //            break;
    //        }
    //    }
        //
        double minDis = 1000000.0;
        if (minS < 0.0)
        {
            for (int i = 0; i < csp.m_s.size(); i++)
            {
                double s0 = csp.m_s[i];
                std::vector<double> p = csp.calc_position(s0);
                double dis = sqrt((pos[0] - p[0])*(pos[0] - p[0])
                    + (pos[1] - p[1])*(pos[1] - p[1]));
                if (dis < minDis)
                {
                    minDis = dis;
                    minS = s0;
                    minSid = i;
                }
            }

            posMin = csp.calc_position(minS);

           // printf("bbbbbb %f %f | %f %f | %f %f\n", pos[0], pos[1], posMin[0], posMin[1], minS, csp.m_s[csp.m_s.size() - 1]);
        }

        if (minSid > 0 && minSid < csp.m_s.size())
        {
            double s0 = csp.m_s[minSid];
            double s1 = csp.m_s[minSid - 1];

            std::vector<double> p0 = csp.calc_position(s0);
            std::vector<double> p1 = csp.calc_position(s1);

            std::vector<double> Q = {x0, y0};
            std::vector<double> Proj = {0.0, 0.0};
            double f = -1.0;

            PointProjectToLine(p0, p1, Q, Proj, f);
            if (f >= 0 && f <= 1)
            {
                posMin = Proj;
                minS = s0 + f*(s1 - s0);
            }
        }

        double yawMin = csp.calc_yaw(minS);
        double fside;
        if (fabs(sin(yawMin + CV_PI / 2.0)) > fabs(cos(yawMin + CV_PI / 2.0)))
        {
            fside = (pos[1] - posMin[1])*sin(yawMin + CV_PI / 2.0);
            fside = (fside == 0)?1.0:fside/fabs(fside);
        }
        else
        {
            fside = (pos[0] - posMin[0])*cos(yawMin + CV_PI / 2.0);
            fside = (fside == 0)?1.0:fside/fabs(fside);
        }

        d = -fside*sqrt((pos[0] - posMin[0])*(pos[0] - posMin[0]) + (pos[1] - posMin[1])*(pos[1] - posMin[1]));
        s = minS;

        //    printf("!!! %f %f   %f   %f %f  %f %f\n", pos[0], pos[1], minS, posMin[0], posMin[1], d, s);
        //    std::vector<double> p = csp.calc_position(s);
        //    printf("%f %f\n", p[0], p[1]);
    }

    void getMaxCollisionAvoidanceOmega(double halfH, double halfW, std::vector<std::vector<float>> pointcloud, double v, double &omegaleftbound, double &omegarightbound)
    {
        omegaleftbound = 0.5;
        omegarightbound = 0.5;

        for (int l = 0; l < pointcloud.size(); l++)
        {
            std::vector<float> pt = pointcloud[l];
            double deltat = 0.5;

            if (pt[0] >= -halfW && pt[0] <= halfW)
            {
                continue;
            }

            std::vector<std::vector<double>> poselist_right;
            std::vector<std::vector<double>> poselist_left;

            for (int i = 0; i < 50; i++)
            {
                double omegai = 0.01*i;
                double dti = omegai*omegai;

                std::vector<double> poseright = {0.0, 0.0, 0.0, 0.0};
                poseright[0] = (v/omegai)*(1 - cos(dti));
                poseright[1] = (v/omegai)*sin(dti);
                poseright[2] = poseright[0] + cos(CV_PI/2.0 - dti);
                poseright[3] = poseright[1] + sin(CV_PI/2.0 - dti);
                poselist_right.push_back(poseright);

                std::vector<double> poseleft = {0.0, 0.0, 0.0, 0.0};
                poseleft[0] = -(v/omegai)*(1 - cos(dti));
                poseleft[1] = (v/omegai)*sin(dti);
                poseleft[2] = poseleft[0] - cos(CV_PI/2.0 - dti);
                poseleft[3] = poseleft[1] + sin(CV_PI/2.0 - dti);
                poselist_left.push_back(poseleft);
            }

            for (int i = 0; i < 50; i++)
            {
                std::vector<double> poseright = poselist_right[i];
                std::vector<double> poseleft = poselist_left[i];

                std::vector<double> p0 = {0.0, 0.0};
                std::vector<double> p1 = {0.0, 0.0};
                std::vector<double> Q = {0.0, 0.0};
                std::vector<double> Proj = {0.0, 0.0};
                double f;
                double d1, d2;

                //right bound
                p0[0] = poseright[0]; p0[1] = poseright[1];
                p1[0] = poseright[2]; p1[1] = poseright[3];
                Q[0] = pt[0]; Q[1] = pt[1];
                PointProjectToLine(p0, p1, Q, Proj, f);
                d1 = sqrt((Q[0] - Proj[0])*(Q[0] - Proj[0]) + (Q[1] - Proj[1])*(Q[1] - Proj[1]));
                d2 = sqrt((p0[0] - Proj[0])*(p0[0] - Proj[0]) + (p0[1] - Proj[1])*(p0[1] - Proj[1]));
                if (d1 < halfW && d2 < halfH)
                {
                    if(0.01*(i - 1) < omegarightbound)
                    {
                        omegarightbound = 0.01*(i - 1);
                    }
                }

                //left bound
                p0[0] = poseleft[0]; p0[1] = poseleft[1];
                p1[0] = poseleft[2]; p1[1] = poseleft[3];
                Q[0] = pt[0]; Q[1] = pt[1];
                PointProjectToLine(p0, p1, Q, Proj, f);
                d1 = sqrt((Q[0] - Proj[0])*(Q[0] - Proj[0]) + (Q[1] - Proj[1])*(Q[1] - Proj[1]));
                d2 = sqrt((p0[0] - Proj[0])*(p0[0] - Proj[0]) + (p0[1] - Proj[1])*(p0[1] - Proj[1]));
                if (d1 < halfW && d2 < halfH)
                {
                    if(0.01*(i - 1) < omegaleftbound)
                    {
                        omegaleftbound = 0.01*(i - 1);
                    }
                }
            }
        }
    }

    void getMinCollisionAvoidanceOmega(double halfH, double halfW, double forwarddis, double v, std::vector<std::vector<float>> pointcloud, int &stop, double &omega)
    {
        double omega_left = 0.0;
        double omega_right = 0.0;
        omega = 0.0;
        stop = false;

        for (int i = 0; i < pointcloud.size(); i++)
        {
            double Qx = pointcloud[i][0];
            double Qy = pointcloud[i][1];

            if (Qy <= halfH || Qy > halfH + forwarddis)
            {
                continue;
            }

            double R;

            if (std::abs(Qx) >= halfW)
            {
                continue;
            }

            R = 0.5*((Qy*Qy - halfH*halfH)/(halfW - std::abs(Qx)) - halfW - std::abs(Qx));

            if (R == 0)
            {
                stop = true;
                return;
            }
            omega = v/R;

            if (Qx >= 0 && omega > omega_left)
            {
               omega_left = omega;
            }

            if (Qx < 0 && omega > omega_right)
            {
               omega_right = omega;
            }
        }


        stop = (omega_left*omega_right > 0);
        omega = (omega_left != 0)?-omega_left:omega_right;
        return;
    }

    double onexsquare(double x, double a1, double sigmax)
    {
        x = x/sigmax;
        if (x != 0)
        {
             return (x/std::abs(x))*a1*x*x/(1 + x*x);
        }
        else
        {
            return 0.0;
        }
    }

    void PID(double vi, double omegai, double vlast, double omegalast, double dtheta, double s, double d, double starget, double &v, double &omega)
    {
        v = vi;
        double sigmas = 0.3;
        if (std::abs(s - starget) < sigmas)
        {
            v = (starget - s)*vi/sigmas;
        }

        omega = omegai;

        double sigmad = 0.3;
        double derivativedd = onexsquare(d, 2.0, 0.5*sigmad);

        double dthetasigma = 5.0*CV_PI/180.0;
        double derivativedtheta = onexsquare(dtheta, 0.5, 0.5*dthetasigma);

        double f = derivativedd + derivativedtheta;
        omega = omegai + f*0.3;

        //fprintf(fd, "%f %f\n", dtheta, s);
    }
}