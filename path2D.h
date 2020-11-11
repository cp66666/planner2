#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "spline2Target.h"

#ifndef PATH_2D_H
#define PATH_2D_H

namespace planner2
{
    //a frenet spline is a parametric curve with parameter t, continuously differentiable
    class frenet_spline
    {
        public:
            virtual double calc_point(double t) = 0;
            virtual double calc_first_derivative(double t) = 0;
            virtual double calc_second_derivative(double t) = 0;
            virtual double calc_third_derivative(double t) = 0;
    };

    //here's an example of a frenet spline
    class cos_poly33: public frenet_spline
    {
        public:
            cos_poly33(double d0, double v0, double vd0, double T, double dhT, double vdT);
            ~cos_poly33();

            double calc_point(double t);
            double calc_first_derivative(double t);
            double calc_second_derivative(double t);
            double calc_third_derivative(double t);

        public:
            double m_d0;
            double m_v0;
            double m_vd0;
            double m_omega;

            double m_a0;
            double m_a1;
            double m_a2;
    };

    //a frenet path is a path one can get position and velocity given frenet coordinate s
    class frenet_path
    {
        public:
            virtual void getPos(double s, double &x, double &y, double &yaw) = 0;
            virtual void getVelocity(double s, double &v, double &omega) = 0;
    };

    //here's an example of frenet path
    class Spline2D: public frenet_path
    {
        public:
            Spline2D();
            Spline2D(std::vector<double> x, std::vector<double> y);
            ~Spline2D();
        public:
            void getPos(double s, double &x, double &y, double &yaw);
            void getVelocity(double s, double &v, double &omega);
        public:
            void setTime(std::vector<double> t);
            void setVelocity(std::vector<double> v, std::vector<double> omega);
            void getTime(double s, double &t);
        public:
            void reset(std::vector<double> x, std::vector<double> y);
            std::vector<double> calc_position(double s);
            std::vector<double> calc_position_test(double s);
            double calc_yaw(double s);

        protected:
            std::vector<double> __calc_s(std::vector<double>x, std::vector<double>y);
            int __search_index(double s);
        public:
            std::vector<double> m_s;
            std::vector<double> m_ds;
            std::vector<double> m_d;
            std::vector<double> m_x;
            std::vector<double> m_y;
            std::vector<double> m_t;
            std::vector<double> m_yaw;
        public:
            std::vector<double> m_v;
            std::vector<double> m_omega;
    };

    //Generate frenet path from motionprimitive and frenet sampling
    //mps is the trajectory of motion primitive
    //frenet spline is the d spline from global path, so that one can goes around the some obstacles
    //v is the desired velocity
    Spline2D calc_global_paths(std::vector<motionprimitive> mps, frenet_spline *lat_qp, double v);

    //one can although calc global paths by given global paths
    //NOT implement yet
    //given frenet_path must iplement getVelocity() method either by record or numerical differential
    //frenet_path* calc_global_paths(frenet_path *path_input, frenet_spline *lat_qp);
}

#endif