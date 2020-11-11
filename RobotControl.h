#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

void RoundTheta(float &theta);

class RobotControl
{
public:
	RobotControl();
	~RobotControl(void);
public:
	bool Start();
    void SetOdometry(float v_in, float omega_in);
	void GetOdometry(float &v_out, float &omega_out);

public:
    double m_time;
	double test1, test2, test3;

protected:
	void Run(void);
	//
	std::thread _task;

	double m_v_in;
	double m_omega_in;

	double m_v;
	double m_omega;

    double dq1 = 0.0f;
    double dq2 = 0.0f;
    double ddq1 = 0.0f;
    double ddq2 = 0.0f;

    double m = 1.0f; //scalable, only I/m, I1/m, motor_max_torque/m matters
    double D = 1.0f; //m, this must has dimension as m, so the result can be m/s
    double R = D/5.0f;
    double I = m*D*D/8.0;
    double I1 = I/10.0f;
    double I2 = I/10.0f;

    double motor_sigma = 5;
    double motor_max_torque = 1000*(I1 + I*R*R/(D*D) + 0.5*m*R*R);

    double N_control_circle = 10;
    double dt = 0.02/N_control_circle; //s, this must has dimension as s, so the result can be m/s

	bool m_bSetOdemetryEvent;
};

typedef struct stRobotPG
{
    float x;
    float y;
    float theta;
    float pan;
    float tilt;
    //
    stRobotPG()
    {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        pan = 0.0f;
        tilt = 0.0f;
    }
    stRobotPG(float x_, float y_, float theta_)
    {
        x = x_;
        y = y_;
        theta = theta_;
        pan = 0.0f;
        tilt = 0.0f;
    }
    void initdefault()
    {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        pan = 0.0f;
        tilt = 0.0f;
    }
}stRobotPG;

class CRobotSimulator
{
public:
	CRobotSimulator(RobotControl *robotControl, stRobotPG startPG, int dt, float robotWidth);
	~CRobotSimulator(void);
public:
	void Start();
	stRobotPG GetRealPG();
    void JumpTo(stRobotPG pg);

protected:
	void run(void);
protected:
	std::thread _task;
	int m_dt; //in milliseconds
	float m_robotWidth;
	stRobotPG m_realPG;
	RobotControl *m_robotControl;
	float m_v;
	float m_omega;
	std::vector<float> m_vrands1;
	std::vector<float> m_vrands2;
};
