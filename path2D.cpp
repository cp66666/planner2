#include "path2D.h"

#ifndef SPLINE_DT
#define SPLINE_DT 0.5
#endif

using namespace Eigen;

//cos poly 33 spline
namespace planner2
{
    cos_poly33::cos_poly33(double d0, double v0, double vd0, double T, double dhT, double vdT)
    {
        m_d0 = d0;
        m_v0 = v0;
        m_vd0 = vd0;
        m_omega = 0.1; // = 2*CV_PI/T maybe better
        double hT = T/2.0;

        Matrix3d A;
        A << T*T, T*T*T, (v0/m_omega)*(1 - cos(m_omega*T)),
        hT*hT, hT*hT*hT, (v0/m_omega)*(1 - cos(m_omega*hT)),
        2*T, 3*T*T, v0*sin(m_omega*T);

        Vector3d b(-d0 - m_vd0*T, -d0 - m_vd0*hT + dhT, vdT - vd0);

        Vector3d C = A.colPivHouseholderQr().solve(b);
        m_a0 = C(0);
        m_a1 = C(1);
        m_a2 = C(2);
       // printf("!!! %f %f %f\n", m_a0, m_a1, m_a2);
    }


    cos_poly33::~cos_poly33()
    {

    }

    double cos_poly33::calc_point(double t)
    {
        double xt = m_d0 + m_vd0*t + m_a0*t*t + m_a1*t*t*t + m_a2*(m_v0/m_omega)*(1-cos(m_omega*t));
        return xt;
    }


    double cos_poly33::calc_first_derivative(double t)
    {
        double xt = m_vd0 + 2*m_a0*t + 3*m_a1*t*t + m_a2*m_v0*sin(m_omega*t);
        return xt;
    }
    double cos_poly33::calc_second_derivative(double t)
    {
        double xt = 2*m_a0 + 6*m_a1*t + m_a2*m_v0*m_omega*cos(m_omega*t);
        return xt;
    }

    double cos_poly33::calc_third_derivative(double t)
    {
        double xt = 6*m_a1 - m_a2*m_v0*m_omega*m_omega*sin(m_omega*t);
        return xt;
    }

    //frenet path
    Spline2D::Spline2D()
    {
    }

    Spline2D::Spline2D(std::vector<double> x, std::vector<double> y)
    {
        m_x = x;
        m_y = y;

        m_s.clear();
        m_ds.clear();

        m_s = __calc_s(x, y);
        m_d.resize(m_s.size());
    }
    Spline2D::~Spline2D()
    {

    }

    void Spline2D::reset(std::vector<double> x, std::vector<double> y)
    {
        m_x = x;
        m_y = y;

        m_s.clear();
        m_ds.clear();

        m_s = __calc_s(x, y);
    }

    void Spline2D:: setTime(std::vector<double> t)
    {
        m_t = t;
    }

    void Spline2D::setVelocity(std::vector<double> v, std::vector<double> omega)
    {
        m_v = v;
        m_omega = omega;
    }

    void Spline2D::getPos(double s, double &x, double &y, double &yaw)
    {
        std::vector<double> pos = calc_position(s);
        x = pos[0];
        y = pos[1];
        yaw = calc_yaw(s);
    }

    void Spline2D::getVelocity(double s, double &v, double &omega)
    {
        int i = __search_index(s);
        double prog = (s - m_s[i])/(m_s[i+1] - m_s[i]);
        double dv = m_v[i+1] - m_v[i];
        double domega = m_omega[i+1] - m_omega[i];

        v = m_v[i] + prog*dv;
        omega = m_omega[i] + prog*domega;

        return;
    }

    void Spline2D::getTime(double s, double &t)
    {
        int i = __search_index(s);
        double prog = (s - m_s[i])/(m_s[i+1] - m_s[i]);
        double dt = m_t[i+1] - m_t[i];

        t = m_t[i] + prog*dt;
        return;
    }

    std::vector<double> Spline2D::calc_position(double s)
    {
        int i = __search_index(s);

        if (i == m_s.size() - 1)
        {
            std::vector<double> pos;

            double x = m_x[i];
            double y = m_y[i];

            pos.push_back(x);
            pos.push_back(y);

            return pos;
        }

        double prog = (s - m_s[i])/(m_s[i+1] - m_s[i]);
        double dx = m_x[i+1] - m_x[i];
        double dy = m_y[i+1] - m_y[i];

        double x = m_x[i] + prog*dx;
        double y = m_y[i] + prog*dy;

        std::vector<double> pos;
        pos.push_back(x);
        pos.push_back(y);

        return pos;
    }

    double Spline2D::calc_yaw(double s)
    {
        int i = __search_index(s);
        double prog = (s - m_s[i])/(m_s[i+1] - m_s[i]);
        double yaw;

        if (i == m_s.size() - 1)
        {
            yaw = m_yaw[i];
        }
        else if (i == 0)
        {
            double yaw2 = m_yaw[i+1];
            double yaw1 = m_yaw[i];
            yaw = yaw1*(1 - prog) + yaw2*prog;
        }
        else
        {
            double yaw0 = m_yaw[i - 1];
            double yaw1 = m_yaw[i];
            double yaw2 = m_yaw[i + 1];
            if (prog < 0.5)
            {
                yaw = (1 - (0.5 - prog))*yaw1 + (0.5 - prog)*yaw0;
            }
            else
            {
                yaw = (1 - (prog - 0.5))*yaw1 + (prog - 0.5)*yaw2;
            }
        }
        return yaw;
    }

    std::vector<double> Spline2D::__calc_s(std::vector<double>x, std::vector<double>y)
    {
        std::vector<double> dx;
        for (int i = 0; i < x.size() - 1; i++)
        {
            dx.push_back(x[i + 1] - x[i]);
        }

        std::vector<double> dy;
        for (int i = 0; i < y.size() - 1; i++)
        {
            dy.push_back(y[i + 1] - y[i]);
        }

        std::vector<double> s;
        s.push_back(0.0f);
        int j = 0;

        for (int i = 0; i < dx.size(); i++)
        {
            double _ds = sqrt(dx[i]*dx[i] + dy[i]*dy[i]);
            m_ds.push_back(_ds);
            s.push_back(s[j] + _ds);
            j++;
        }
        return s;
    }

    int Spline2D::__search_index(double s)
    {
        int i;
        for (i = 0; i < m_s.size(); i++)
        {
            if (s < m_s[i])
            {
                break;
            }
        }

        return i - 1;
    }

    void calc_global_paths(Spline2D csp, frenet_spline *lat_qp, double Ttotal, Spline2D &path)
    {
        Spline2D fp = path;

        for (int j = 0; j < csp.m_t.size(); j++)
        {
            double t = csp.m_t[j];
            double d;

            if (lat_qp != NULL)
            {
                double t_ = t/Ttotal;
                d = lat_qp->calc_point(t_);
            }
            else
            {
                d = 0.0;
            }
                   // printf("%f %f \n", t, d);

            fp.m_d.push_back(d);
            fp.m_v.push_back(csp.m_v[j]);
            fp.m_omega.push_back(csp.m_omega[j]);
        }
        //
        double iyaw;
        for (int j = 0; j < csp.m_s.size(); j++)
        {
            double s = csp.m_s[j];
            std::vector<double> pos = csp.calc_position(s);

            double ix = pos[0];
            double iy = pos[1];

            double fx, fy;

            if (j == csp.m_s.size() - 1)
            {
                //don't update iyaw
            }
            else
            {
                double s1 = csp.m_s[j + 1];
                std::vector<double> pos1 = csp.calc_position(s1);
                double dx = pos1[0] - ix;
                double dy = pos1[1] - iy;
                iyaw = atan2(dy, dx);
            }

            double di = fp.m_d[j];
            fx = ix + di * cos(iyaw + CV_PI / 2.0);
            fy = iy + di * sin(iyaw + CV_PI / 2.0);

            fp.m_x.push_back(fx);
            fp.m_y.push_back(fy);
        }

        fp.m_s.push_back(csp.m_s[0]);
        fp.m_t.push_back(csp.m_t[0]);
        for (int j = 0; j < fp.m_x.size() - 1; j++)
        {
            double s_ = fp.m_s[j];
            double dx = fp.m_x[j+1] - fp.m_x[j];
            double dy = fp.m_y[j+1] - fp.m_y[j];

            double s = s_ + sqrt(dx*dx + dy*dy);
            fp.m_s.push_back(s);

            double t_ = fp.m_t[j];
            double dt_ = csp.m_t[j+1] - csp.m_t[j];
            double ds1 = s - s_;
            double ds0 = csp.m_s[j+1] - csp.m_s[j];
            double t;

            if (ds0 != 0.0)
            {
                t = t_ + dt_*ds1/ds0;
            }
            else
            {
                t = t_ + dt_;
            }


            fp.m_t.push_back(t);

            iyaw = atan2(dy, dx);
            fp.m_yaw.push_back(iyaw);
        }
        fp.m_yaw.push_back(iyaw);

        for (int j = 0; j < fp.m_s.size() - 2; j++)
        {
            double dx = fp.m_x[j + 1] - fp.m_x[j];
            double dy = fp.m_y[j + 1] - fp.m_y[j];
            double yaw;
            if (dy == 0 && dx == 0)
            {
                yaw = 0.0;
            }
            else
            {
                yaw = atan2(dy ,dx);
            }

            int j1 = j + 1;
            double dx1 = fp.m_x[j1 + 1] - fp.m_x[j1];
            double dy1 = fp.m_y[j1 + 1] - fp.m_y[j1];
            double yaw1;
            if (dy1 == 0 && dx1 == 0)
            {
                yaw1 = 0.0;
            }
            else
            {
                yaw1 = atan2(dy1 ,dx1);
            }

            double dyaw = yaw1 - yaw;

            if (dyaw > CV_PI)
            {
                dyaw -= 2 * CV_PI;
            }
            else if (dyaw <= -CV_PI)
            {
                dyaw += 2 * CV_PI;
            }
            fp.m_omega[j] = dyaw/(fp.m_t[j + 1] - fp.m_t[j]);
        }
        fp.m_omega.push_back(fp.m_omega[fp.m_omega.size() - 1]);
        fp.m_omega.push_back(fp.m_omega[fp.m_omega.size() - 1]);
        path = fp;
    }

    Spline2D calc_global_paths(std::vector<motionprimitive> mps, frenet_spline *lat_qp, double v)
    {
        //
        //converting motion primitive to readable path
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> thetas;
        std::vector<double> vs;
        std::vector<double> omegas;
        std::vector<double> ts;

        double f = v/0.1;
        double f1 = 1.0/f;

        for (int j = 0; j < mps.size(); j++)
        {
            motionprimitive mp = mps[j];
            mp.print();
            std::vector<std::vector<double>> trajj;
            if (mp.N == 0)
            {
                continue;
            }
            else if (mp.N == 1)
            {
                forward1(mp.xstart, mp.ystart, mp.thetastart, mp.v, mp.omega1, mp.t1, trajj);
            }
            else if (mp.N == 2)
            {
                double t_[2];
                t_[0] = mp.t1;
                t_[1] = mp.t2;
                forward2(mp.xstart, mp.ystart, mp.thetastart, mp.v, mp.omega1,  mp.omega2, t_, trajj);
            }
            else if (mp.N == 3)
            {
                double t_[3];
                t_[0] = mp.t1;
                t_[1] = mp.t2;
                t_[2] = mp.t3;
                forward3(mp.xstart, mp.ystart, mp.thetastart, mp.v, mp.omega1, t_, trajj);
            }
            for (int i = 0; i < trajj.size() - 1; i++)
            {
                xs.push_back(trajj[i][0]);
                ys.push_back(trajj[i][1]);
                thetas.push_back(trajj[i][2]);
                vs.push_back(f*trajj[i][3]);
                omegas.push_back(f*trajj[i][4]);
                ts.push_back(f1*trajj[i][5]);
            }
        }

        //
        //extend global path + 0.4m
        double xt = xs[xs.size() - 1];
        double yt = ys[ys.size() - 1];
        double thetat = thetas[thetas.size() - 1];
        double vt = vs[vs.size() - 1];
        double Ttarget = ts[ts.size() - 1];

        float t4 = 0.5/vt;
        float dt = SPLINE_DT;
        float t = dt;

        while (t < t4)
        {
            double x_ = xt - t*vt*sin(thetat);
            double y_ = yt + t*vt*cos(thetat);
            double v_ = vt;
            double omega_ = 0.0;

            xs.push_back(x_);
            ys.push_back(y_);
            vs.push_back(v_);
            omegas.push_back(omega_);
            ts.push_back(t + Ttarget);

            t += dt;
        }
        double Ttotal = ts[ts.size() - 1];

        //
        //put data in spline2D
        Spline2D csp = Spline2D(xs, ys);
        csp.setTime(ts);
        csp.setVelocity(vs, omegas);

        //
        //add frenet sampling
        Spline2D path;
        calc_global_paths(csp, NULL, Ttotal, path);
        return path;
    }
}