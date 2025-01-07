#ifndef DATACONFIGURATION_H_
#define DATACONFIGURATION_H_

#include <string.h>
#include <cstdio>

#pragma pack(push)  /* push current alignment to stack*/
#pragma pack(4)     /* set alignment to 4 byte boundary*/

struct LoggedData
{
	enum
	{
		JOINT_DOF = 6
	};

	double time;
	double q[JOINT_DOF];
	double qdot[JOINT_DOF];
	double tau[JOINT_DOF];

	LoggedData()
	: time(0.0), q{0.0}, qdot{0.0}, tau{0,0}
	{}

	LoggedData(double time, const double * q, const double * qdot, const double * tau)
	: time(time)
	{
		memcpy(this->q, q, JOINT_DOF*sizeof(double));
		memcpy(this->qdot, qdot, JOINT_DOF*sizeof(double));
		memcpy(this->tau, tau, JOINT_DOF*sizeof(double));
	}

	LoggedData(LoggedData const & data)
	{
		memcpy(this, &data, sizeof(LoggedData));
	}

	LoggedData & operator=(const LoggedData & data)
	{
		memcpy(this, &data, sizeof(LoggedData));
		return (*this);
	}

	void update(double time, const double *q, const double *qdot, const double *tau)
	{
		this->time = time;

		for (int i = 0; i < JOINT_DOF; i++)
		{
			this->q[i] = q[i];
			this->qdot[i] = qdot[i];
			this->tau[i] = tau[i];
		}
	}
};

#pragma pack(pop)   /*restore original alignment from stack*/

#endif /*DATACONFIGURATION_H_*/
