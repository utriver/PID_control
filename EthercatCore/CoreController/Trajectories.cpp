/*
 * Trajectories.cpp
 *
 *  Created on: 2020. 10. 16.
 *      Author: MesAbay
 */

#include "Controllers.h"


using namespace std;


bool Controllers::read_trajectory(const char *trajectory_path) 
{
  fstream file_traj_pos, file_traj_vel, file_traj_acc;

  sprintf(pos_trajectory, "%s%s.csv", trajectory_path, "traj_pos_pos1");
  sprintf(vel_trajectory, "%s%s.csv", trajectory_path, "traj_vel_pos1");
  sprintf(acc_trajectory, "%s%s.csv", trajectory_path, "traj_acc_pos1");

  file_traj_pos.open(pos_trajectory);
	file_traj_vel.open(vel_trajectory);
	file_traj_acc.open(acc_trajectory);

	string line;

  try
  {
 
		// Load trajectory
		while (getline( file_traj_pos, line,'\n'))
		{
		  istringstream templine(line);
		  string data;
		  while (getline( templine, data,','))
		  {
			  _qdes_traj_pos.push_back(atof(data.c_str()));
		  }
		}

		if (!file_traj_pos.is_open())
		{
			throw ERROR_POS_FILE_NOT_FOUND;
		}

		while (getline( file_traj_vel, line,'\n'))
		{
		  istringstream templine(line);
		  string data;
		  while (getline( templine, data,','))
		  {
			  _qdes_traj_vel.push_back(atof(data.c_str()));
		  }
		}

		if (!file_traj_vel.is_open())
		{
			throw ERROR_VEL_FILE_NOT_FOUND;
		}

		while (getline( file_traj_acc, line,'\n'))
		{
		  istringstream templine(line);
		  string data;
		  while (getline( templine, data,','))
		  {
			  _qdes_traj_acc.push_back(atof(data.c_str()));
		  }
		}

		if (!file_traj_acc.is_open())
		{
			throw ERROR_ACC_FILE_NOT_FOUND;
		}

		max_traj_size = _qdes_traj_pos.size();
	//}catch (std::exception & e)
	}catch (int exception)
	{
		//std::cerr << "Exception caught : " <<e.what() << std::endl;
		switch(exception)
		{
		case 1:
			printf("\n No position trajectory uploaded...\n");
			break;
		case 2:
			printf("\n No velocity trajectory uploaded...\n");
			break;
		case 3:
			printf("\n No acceleration trajectory uploaded...\n");
			break;
		default:
			break;
		}
		exit(1);
	}

	file_traj_pos.close();
	file_traj_vel.close();
	file_traj_acc.close();

	return 1;  
}

bool Controllers::read_step_trajectory(char *trajectory_path)
{
  fstream file_traj_pos;

  sprintf(pos_trajectory,"%s%s",trajectory_path,"traj_pos.csv");

  file_traj_pos.open(pos_trajectory);

	string line;

  try
  { 
		// Load trajectory
		while (getline( file_traj_pos, line,'\n'))
		{
		  istringstream templine(line);
		  string data;
		  while (getline( templine, data,','))
		  {
			  _qdes_traj_pos.push_back(atof(data.c_str()));
		  }
		}

		if (!file_traj_pos.is_open())
		{
			throw ERROR_POS_FILE_NOT_FOUND;
		}

		max_traj_size = _qdes_traj_pos.size();
	//}catch (std::exception & e)
	}catch (int exception)
	{
		//std::cerr << "Exception caught : " <<e.what() << std::endl;
		switch(exception)
		{
		case 1:
			printf("\n No position trajectory uploaded...\n");
			break;
		default:
			break;
		}
		exit(1);
	}

	file_traj_pos.close();

	return 1;  
}

double Controllers::gen_lspb_trajectory(bool isLspbOn)
{
  double lspb_wave_pos = 0;
	double lspb_wave_vel = 0;
	double lspb_wave_acc = 0;
	static int cnt_traj = 0;
	static int isPtraj_acc = 0;

	if (isLspbOn == true)
	{
		if (cnt_traj>=max_traj_size) cnt_traj = 0;
		lspb_wave_pos = _qdes_traj_pos[cnt_traj];
		lspb_wave_vel = _qdes_traj_vel[cnt_traj];
		lspb_wave_acc = _qdes_traj_acc[cnt_traj];
		cnt_traj++;
	}
	else
	{

	}

	if (isLspbOn == true)
	{
		if (lspb_wave_acc>0)
		{
			traj_phase = 0;
			isPtraj_acc = 1;

		}else if(lspb_wave_acc<0)
		{
			traj_phase = 0;
			isPtraj_acc = 0;
		}else
		{
			if (isPtraj_acc == 1)
			{
				traj_phase=1;
			}else
			{
				traj_phase=2;
			}
		}
	}else
	{

	}

	return lspb_wave_pos;
}

void Controllers::gen_lspb_trajectory(bool isLspbOn, double & lspb_wave_pos, double & lspb_wave_vel,double & lspb_wave_acc)
{

	static int cnt_traj = 0;
	static int isPtraj_acc = 0;

	if (isLspbOn == true)
	{
		if (cnt_traj>=max_traj_size) cnt_traj = 0;
		lspb_wave_pos = _qdes_traj_pos[cnt_traj];
		lspb_wave_vel = _qdes_traj_vel[cnt_traj];
		lspb_wave_acc = _qdes_traj_acc[cnt_traj];
		cnt_traj++;
	}
	else
	{

	}

	if (isLspbOn == true)
	{
		if (lspb_wave_acc>0)
		{
			traj_phase = 0;
			isPtraj_acc = 1;

		}else if(lspb_wave_acc<0)
		{
			traj_phase = 0;
			isPtraj_acc = 0;
		}else
		{
			if (isPtraj_acc == 1)
			{
				traj_phase=1;
			}else
			{
				traj_phase=2;
			}
		}
	}else
	{

	}

//	return 0;
}

double Controllers::gen_step_trajectory(bool isLspbOn)
{
  double lspb_wave_pos = 0;
	static int cnt_traj = 0;

	if (isLspbOn == true)
	{
		if (cnt_traj>=max_traj_size) cnt_traj = 0;
		lspb_wave_pos = _qdes_traj_pos[cnt_traj];
		cnt_traj++;
	}
	else
	{

	}

	if (isLspbOn == true)
	{
		
	}else
	{

	}

	return lspb_wave_pos;
}
