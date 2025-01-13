/*
 * Controllers.h
 *
 *  Created on: 2020. 10. 16.
 *      Author: MesAbay
 */


#ifndef CORECONTROLLER_CONTROLLERS_H_
#define CORECONTROLLER_CONTROLLERS_H_

#pragma once

#include "../Library/eigen/Eigen/Eigen"
#include "../Library/eigen/Eigen/SVD"
#include "../define.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <math.h>

using namespace std;

class Controllers
{
private:
  double _delT;
public:
  	enum
	{
		NUM_ACT = NUM_AXIS,
	};
  
  typedef	Eigen::Matrix<double, NUM_ACT, 1> JointVec;
	typedef Eigen::Matrix<double, NUM_ACT, NUM_ACT> JointMat;

  //PID controller
  JointVec _pid_eint_dr;
	JointVec _pid_e_dr_prev;
	JointVec _pid_edot_dr_prev;

	JointVec e_dr;
	JointVec edot_dr;


public:
  Controllers(/* args */);
  ~Controllers();

public:
  JointVec PIDcontroller(JointMat kp, JointMat ki, JointMat kd, JointVec y, JointVec y_des);
  JointVec PIDcontroller(JointMat kp, JointMat ki, JointMat kd, JointVec y, JointVec y_des, JointVec ydot, JointVec ydot_des);
  JointVec filteredDerivative(JointVec input_prev, JointVec input_present, JointVec output_prev, double cutoff);
  
// encoder to joint and vs. conversion
public:
  void joint_to_encoder(JointVec joint, INT32 *enc);
  void encoder_to_joint(INT32 *enc, JointVec joint);

public:
  bool read_trajectory(char *trajectory_path);
  bool read_step_trajectory(char *trajectory_path);
  double gen_lspb_trajectory(bool isLspbOn);
  void gen_lspb_trajectory(bool isLspbOn, double & lspb_wave_pos, double & lspb_wave_vel, double & lspb_wave_acc);
  double gen_step_trajectory(bool isLspbOn);

public:
  char *pos_trajectory;
  char *vel_trajectory;
  char *acc_trajectory;

  vector<double> _qdes_traj_pos, _qdes_traj_vel, _qdes_traj_acc;
 	unsigned int max_traj_size = 0;
  int traj_phase = -1;


};


#endif /* CORECONTROLLER_CONTROLLERS_H_ */
