#include "RobotControl.h"
#include <Eigen/Dense>
#include <chrono>

using namespace Eigen;

void RoundTheta(float &theta)
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

void RoundTheta(double &theta)
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


RobotControl::RobotControl()
{
    m_omega_in = 0.0f;
	m_omega = 0.0f;
	m_v_in = 0.0f;
	m_v = 0.0f;
}

RobotControl::~RobotControl(void)
{

}

bool RobotControl::Start()
{
	_task = (std::thread(std::bind(&RobotControl::Run, this)));
	m_bSetOdemetryEvent = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

	return true;
}

void RobotControl::Run(void)
{
    auto t1=std::chrono::steady_clock::now();

	while (true)
	{
		while (!m_bSetOdemetryEvent)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		m_bSetOdemetryEvent = false;

        float tdq1 = (m_v_in - m_omega_in*D*0.5)/R;
        float tdq2 = (m_v_in + m_omega_in*D*0.5)/R;

        for (int i = 0; i < N_control_circle; i++)
        {
            double delta_q1 = tdq1 - dq1;
            double delta_q2 = tdq2 - dq2;
            //printf("target %f |", m_v_in, m_omega_in);
            //printf("delta %f %f |", delta_q1/motor_sigma, delta_q2/motor_sigma);

            double N1 = motor_max_torque*(1.0/(1 + exp(-delta_q1/motor_sigma)) - 0.5);
            double N2 = motor_max_torque*(1.0/(1 + exp(-delta_q2/motor_sigma)) - 0.5);

           // printf("N %f %f %f |", motor_max_torque, N1, N2);

            //m_v = m_v_in;
            //m_omega = m_omega_in;

            Matrix2d A;
            A << I1 + I*R*R/(D*D) + 0.5*m*R*R, - I*R*R/(D*D) + 0.5*m*R*R,
                - I*R*R/(D*D) + 0.5*m*R*R, I2 + I*R*R/(D*D) + 0.5*m*R*R;

            Vector2d b(N1, N2);
            Vector2d C = A.colPivHouseholderQr().solve(b);

            ddq1 = C(0);
            ddq2 = C(1);

            //printf("ddq %f %f |", ddq1, ddq2);

            dq1 = dq1 + ddq1*dt;
            dq2 = dq2 + ddq2*dt;

            //auto t3 = std::chrono::steady_clock::now();
            //double dt = std::chrono::duration<double, std::milli>(t3 - t1).count();

            //printf("dq %f %f |", dq1, dq2);

            m_v = 0.5*(dq1*R + dq2*R);
            m_omega = (dq2*R - dq1*R)/D;
           // printf("v omega %f %f\n", v, omega);
        }
        auto t2 = std::chrono::steady_clock::now();
        m_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
    }
}

void  RobotControl::SetOdometry(float v_in, float omega_in)
{
    m_bSetOdemetryEvent = true;
    m_v_in = v_in;
    m_omega_in = -omega_in;
}

void RobotControl::GetOdometry(float &v_out, float &omega_out)
{
	v_out = m_v;
	omega_out = m_omega;
}

CRobotSimulator::CRobotSimulator(RobotControl *robotControl, stRobotPG startPG, int dt, float robotWidth)
{
	m_realPG = startPG;
	m_dt = dt;
	m_robotWidth = robotWidth;

	m_robotControl = robotControl;
}

CRobotSimulator::~CRobotSimulator(void)
{
}

void CRobotSimulator::Start()
{
	m_v = 0.0f;
	m_omega = 0.0f;
	_task = (std::thread(std::bind(&CRobotSimulator::run, this)));
}

void CRobotSimulator::run()
{
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(m_dt));

		m_robotControl->GetOdometry(m_v, m_omega);

		float omega, v;
		omega = m_omega*(1.0f);
		v = m_v*(1.0f);

		float radian = omega*0.001f*m_dt;
		RoundTheta(radian);

		float delta[2];
		//delta[0] = -sin(m_realPG.theta) * v * 0.001f*m_dt;
		//delta[1] = cos(m_realPG.theta) * v * 0.001f*m_dt;
		delta[0] = cos(m_realPG.theta) * v * 0.001f*m_dt;
		delta[1] = sin(m_realPG.theta) * v * 0.001f*m_dt;

		m_realPG.x += delta[0];
		m_realPG.y += delta[1];
		m_realPG.theta -= radian;

		RoundTheta(m_realPG.theta);
		//printf("%f %f %f\n", v, omega, m_realPG.theta);
		//m_robotControl->SetOdometry(m_v, m_omega);
	}
}

void CRobotSimulator::JumpTo(stRobotPG pg)
{
    m_realPG = pg;
}

stRobotPG CRobotSimulator::GetRealPG()
{
	return(m_realPG);
}
