/*
 * Controllers.cpp
 *
 *  Created on: 2020. 10. 16.
 *      Author: MesAbay
 */

#include "Controllers.h"
#define DEG2RAD PI/180;
using namespace std;

typedef Controllers::JointVec JointVec;

Controllers::Controllers()
:_delT(0.001)
{
	_pid_eint_dr.setZero();
	_pid_e_dr_prev.setZero();
	_pid_edot_dr_prev.setZero();
	e_dr.setZero();
	edot_dr.setZero();

	pos_trajectory = new char[100];
	vel_trajectory = new char[100];
	acc_trajectory = new char[100];
}

Controllers::~Controllers()
{

}

JointVec Controllers::filteredDerivative(JointVec input_prev, JointVec input_present, JointVec output_prev, double cutoff)
{
	double ALPHA = ( (2 * cutoff * _delT) / (2 + cutoff*_delT));
	JointVec output_present = ALPHA * ( (input_present - input_prev)/_delT) + (1 - ALPHA) * output_prev;
	//cout<<"input_present:"<<input_present<<endl<<"input_prev:"<<input_prev<<endl;

	return output_present;
}

JointVec Controllers::PIDcontroller(JointMat kp, JointMat ki, JointMat kd, JointVec y, JointVec y_des)
{
	e_dr = y_des - y;
	edot_dr = filteredDerivative(_pid_e_dr_prev, e_dr, _pid_edot_dr_prev, 20);

	_pid_eint_dr += (e_dr + _pid_e_dr_prev)*_delT;
	_pid_e_dr_prev = e_dr;
	_pid_edot_dr_prev = edot_dr;

	return (kp*e_dr + ki*_pid_eint_dr + kd*edot_dr);
}

JointVec Controllers::PIDcontroller(JointMat kp, JointMat ki, JointMat kd, JointVec y, JointVec y_des, JointVec ydot, JointVec ydot_des)
{
	e_dr = y_des - y;
	edot_dr = ydot_des - ydot;


	_pid_eint_dr += (e_dr + _pid_e_dr_prev)*_delT;
	_pid_e_dr_prev = e_dr;
	_pid_edot_dr_prev = edot_dr;

	return (kp*e_dr + ki*_pid_eint_dr + kd*edot_dr);
}

// Controllers.cpp

double Controllers::filteredDerivative(const double& input_prev, const double& input_present, const double& output_prev, double cutoff)
{
    double ALPHA = ((2 * cutoff * _delT) / (2 + cutoff * _delT));
    double derivative = (input_present - input_prev) / _delT;
    double filtered = ALPHA * derivative + (1 - ALPHA) * output_prev;
    return filtered;
}

