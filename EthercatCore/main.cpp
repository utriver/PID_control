// Automatically generated realtime application source file for STEP platforms
//
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP).
//
// Copyright (C) 2013-2015 Neuromeka <http://www.neuromeka.com>

//-system-/////////////////////////////////////////////////////////////////
#ifndef __XENO__
#define __XENO__
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

#include "EcatSystem/SystemInterface_EtherCAT_EthercatCore.h"
#include "EcatDataSocket.h"
#include "EcatControlSocket.h"

#include "ServoAxis.h"
#include "DataLogger/DataLogger.h"

// PID gian value
#define NUM_AXIS	(1)		//Modify this number to indicate the actual number of motor on the network
#define KP 1.0
#define KI 0.5
#define KD 0.1

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME		(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

#define DEMO_MODE_TORQUE		1
#define DEMO_MODE_POSITION		2
#define DEMO_MODE_GUI			3


#define EncoderResolution		65536
#define GearRatio               121
#define deg2rad PI/180
////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 1000

static int sampling_time = 5;	// Data is sampled every 5 cycles.
volatile int sampling_tick = 0;

struct LOGGING_PACK
{
	double Time;
	INT32 	ActualPos[NUM_AXIS];
	INT32 	ActualVel[NUM_AXIS];
};

unsigned int frontIdx = 0, rearIdx = 0;
LOGGING_PACK _loggingBuff[MAX_BUFF_SIZE];
/////////////////////////////////////////

// Cycle time in nanosecond
unsigned int cycle_ns = 1000000; /* 1 ms */

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;


// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// EtherCAT System interface object
SystemInterface_EtherCAT_EthercatCore _systemInterface_EtherCAT_EthercatCore;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> guicontrolsocket;

// Demo Mode
int demo_mode = DEMO_MODE_TORQUE;
// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;
double q[NUM_AXIS] = {0,};
double qdot[NUM_AXIS] = {0,};
double core_tor[NUM_AXIS] = {0,};

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;
float degree =0;
int InitFlag[NUM_AXIS] = {0};
double qdes[NUM_AXIS] = {0}; // Desired position
double qdotdes[NUM_AXIS] = {0}; // Desired position
double qdotdotdes[NUM_AXIS] = {0}; // Desired position

// EtherCAT Data (in pulse)
INT32 	ZeroPos[NUM_AXIS] = {0};
UINT16	StatusWord[NUM_AXIS] = {0};
INT32 	ActualPos[NUM_AXIS] = {0};
INT32 	ActualVel[NUM_AXIS] = {0};
INT16 	ActualTor[NUM_AXIS] = {0};
UINT32	DataIn[NUM_AXIS] = {0};
UINT8	ModeOfOperationDisplay[NUM_AXIS] = {0};

double integral_error[NUM_AXIS] = {0};
double previous_error[NUM_AXIS] = {0};
double computed_torque[NUM_AXIS] = {0};

INT32 	TargetPos[NUM_AXIS] = {0};
INT32 	TargetVel[NUM_AXIS] = {0};
INT16 	TargetTor[NUM_AXIS] = {0};
UINT32 	DataOut[NUM_AXIS] = {0};
UINT8 	ModeOfOperation[NUM_AXIS] = {0};
UINT16	Controlword[NUM_AXIS] = {0};

DataLogger _dataLogger;
unsigned int _logCnt;
double _time;
#define LOG_DATA_SAVE_PERIOD  30*4000

///// SDO Access /////////



//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

/****************************************************************************/

// Xenomai RT tasks
RT_TASK EthercatCore_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK gui_task;

// For RT thread management
static int run = 1;
unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;
#define min_time	0
#define max_time	100000
#define hist_step	(100)
unsigned int histdata[hist_step+1];
unsigned int interval_size=350;

// Signal handler for CTRL+C
void signal_handler(int signum);

int initAxes()
{
	
	const int gearRatio[NUM_AXIS] = {0};
	const int pulsePerRevolution[NUM_AXIS] = {0};
	const double ratedTau[NUM_AXIS] = {0};
	const int dirQ[NUM_AXIS] = {0};
	const int dirTau[NUM_AXIS] = {0};
	const int zeroPos[NUM_AXIS] = {0};
	

	for (int i = 0; i < NUM_AXIS; i++)
	{
		Axis[i].setGearRatio(1);
		Axis[i].setPulsePerRevolution(1);
		Axis[i].setRatedTau(1);

		Axis[i].setDirQ(1);
		Axis[i].setDirTau(1);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
		
		_systemInterface_EtherCAT_EthercatCore.setServoOn(i);
	}
	
	return 1;
}
void reset_fault(int slave_index) {
    int status_word = 0;
    int control_word = 0;

    // 1. Read StatusWord to check the fault state
    _systemInterface_EtherCAT_EthercatCore.readBuffer(0x6041, &status_word);
    printf("Initial StatusWord: 0x%X\n", status_word);

    // Check if the Fault bit (0x08) is set
    if (status_word & 0x08) {
        printf("Fault detected. Resetting...\n");

        // 2. Send Fault Reset (ControlWord = 0x80)
        control_word = 0x80;
        _systemInterface_EtherCAT_EthercatCore.writeBuffer(0x6040, &control_word);

        // Wait for status to update
        usleep(100000); // 100ms delay

        // 3. Check if the fault reset was successful
        _systemInterface_EtherCAT_EthercatCore.readBuffer(0x6041, &status_word);
        printf("StatusWord after Fault Reset: 0x%X\n", status_word);

        // Confirm Fault is cleared (StatusWord should no longer have 0x08 set)
        if (status_word & 0x08) {
            printf("Fault reset failed. Check hardware or configuration.\n");
            return;
        }

        // 4. Transition to "Operation Enabled"
        printf("Recovering to Operation Enabled...\n");

        // Switch On
        control_word = 0x06;
        _systemInterface_EtherCAT_EthercatCore.writeBuffer(0x6040, &control_word);
        usleep(100000);

        // Enable Voltage
        control_word = 0x07;
        _systemInterface_EtherCAT_EthercatCore.writeBuffer(0x6040, &control_word);
        usleep(100000);

        // Operation Enabled
        control_word = 0x0F;
        _systemInterface_EtherCAT_EthercatCore.writeBuffer(0x6040, &control_word);
        usleep(100000);

        // 5. Verify StatusWord is Operation Enabled
        _systemInterface_EtherCAT_EthercatCore.readBuffer(0x6041, &status_word);
        printf("Final StatusWord: 0x%X\n", status_word);

        if ((status_word & 0x04) && (status_word & 0x08) == 0) {
            printf("Slave is now in Operation Enabled state.\n");
        } else {
            printf("Failed to transition to Operation Enabled. Current StatusWord: 0x%X\n", status_word);
        }
    } else {
        printf("No fault detected. Current StatusWord: 0x%X\n", status_word);
    }
}

void saveLogData()
{
	if (datasocket.hasConnection() && sampling_tick-- == 0)
	{
		sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

		if (rearIdx < MAX_BUFF_SIZE)
		{
			_loggingBuff[rearIdx].Time = gt;
			for (int i=0; i<NUM_AXIS; i++)
			{
				_loggingBuff[rearIdx].ActualPos[i] = ActualPos[i];
				_loggingBuff[rearIdx].ActualVel[i] = ActualVel[i];
			}
			rearIdx++;
		}
	}
}
double Cnt2Nm =  0.2228457; //gear ratio (121:1) is already considered
double Nm2Cnt = 1/Cnt2Nm;
void ethercat2physical()
{
	for (int i=0; i<NUM_AXIS; ++i)
	{
		double temp_q, temp_qdot;
		switch (i)
		{
		case 0:
		
			q[i] = (double) (ActualPos[i] - ZeroPos[i])*360/7929856;
			qdot[i] = (double) ActualVel[i]*360/7929856;
			core_tor[i] = (double) ActualTor[i]*Cnt2Nm;
			break;
		// case 1:
		// 	temp_q = (double) (ActualPos[i] - ZeroPos[i])*360;
		// 	temp_q /= EncoderResolution;
		// 	temp_q /= GearRatio;
		// 	q[i] = temp_q*deg2rad;
		// 	break;
		// case 2:
		// 	temp_q = (double) (ActualPos[i] - ZeroPos[i])*360;
		// 	temp_q /= EncoderResolution;
		// 	temp_q /= GearRatio;
		// 	q[i] = temp_q*deg2rad;
		// 	break;
		}
	}
	
}
double control_signal=0;
double maxtorq = 200;
double physical2ethercat(double control_signal)
{
	for (int i=0; i<NUM_AXIS; ++i)
	{
		double temp_tau, temp_qdot;
		
			// temp_q = q[i]/deg2rad;
			// temp_q *= GearRatio;
			// temp_q *= EncoderResolution;
			// TargetPos[i] = (INT32) temp_q + ZeroPos[i];
			// change torque limit based on the max torque of the core
		if (control_signal > maxtorq ) control_signal = maxtorq;
		else if (control_signal < -maxtorq) control_signal = -maxtorq;
		
		temp_tau = control_signal * Nm2Cnt;
		return temp_tau;
		break;
		// case 1:
		// 	temp_q = q[i]/deg2rad;
		// 	temp_q *= GearRatio;
		// 	temp_q *= EncoderResolution;
		// 	TargetPos[i] = (INT32) temp_q + ZeroPos[i];
		// 	break;
		// case 2:
		// 	temp_q = q[i]/deg2rad;
		// 	temp_q *= GearRatio;
		// 	temp_q *= EncoderResolution;
		// 	TargetPos[i] = (INT32) temp_q + ZeroPos[i];
		// 	break;
		
	}
}
/****************************************************************************/
// int compute()
// {
// 	if (demo_mode == DEMO_MODE_TORQUE)
// 	{
// 		// CST
// 		for (int i=0; i<NUM_AXIS; ++i)
// 		{
// 			if (system_ready)
// 				TargetTor[i]= (int) degree*(sin(PI2*f*gt));
// 			else
// 			{
// 				_systemInterface_EtherCAT_EthercatCore.setServoOn(i);	
// 				TargetTor[i]=0;
// 			}
// 		}
		
// 	}
// 	else if (demo_mode == DEMO_MODE_POSITION)
// 	{
// 		//CSP
// 		for (int i=0; i<NUM_AXIS; ++i)
// 		{
// 			if (system_ready)
// 			{
// 				TargetPos[i]=(int) (sine_amp*(sin(PI2*f*gt))) + ZeroPos[i];
// 			}
// 			else if ((InitFlag[i]==0) && (ActualPos[i]!=0))
// 			{
// 				_systemInterface_EtherCAT_EthercatCore.setServoOn(i);	
// 				TargetPos[i] = ZeroPos[i] = ActualPos[i];
// 				InitFlag[i] = 1;
// 			}
// 		}

// 	}
// 	else
// 	{
// 		// For Command Control	
// 		for (int i=0; i<NUM_AXIS; ++i)
// 		{
// 			Axis[i].setCurrentPosInCnt(ActualPos[i]);
// 			Axis[i].setCurrentVelInCnt(ActualVel[i]);
// 			Axis[i].setCurrentTorInCnt(ActualTor[i]);
// 			Axis[i].setDataIn(DataIn[i]);
			
// 			Axis[i].setCurrentTime(gt);
					
// 			// Init Trajectory
// 			if ((InitFlag[i] == 0) && (ModeOfOperation[i] == OP_MODE_CYCLIC_SYNC_POSITION))
// 			{
// 				if ((ActualPos[i] != 0) || (StatusWord[i] != 0))
// 				{
// 					TargetPos[i] = ActualPos[i];
// 					Axis[i].setTarPosInCnt(ActualPos[i]);
// 					InitFlag[i] = 1;
// 				}			
// 			}
// 			else
// 				_systemInterface_EtherCAT_EthercatCore.setServoOn(i);		
			
// 			TargetPos[i] = Axis[i].getDesPosInCnt();
// 			TargetVel[i] = Axis[i].getDesVelInCnt();
// 			TargetTor[i] = Axis[i].getDesTorInCnt();
// 			DataOut[i] = Axis[i].getDataOut();
// 		}
// 	}
	
// 	return 0;
// }
double error_integral = 0;
double kp=1, ki=0.1, kd=0.1;
double error, error_dot;

double pid_control(double qdes, double q, double qdot, double qdotdes, double &computed_torque) {
    error = qdes - q;
	error_dot = qdotdes - qdot;

//	computed_torque = kp * error - kd * qdot;
	computed_torque = kp * error + kd * error_dot + ki * error_integral;
	error_integral += 0.001 * error;
	return computed_torque;

}

void LowPassDerivative(const double & input_prev, const double& input_present, const double& output_prev, const double& cutoff , double& output){
	double _delT=0.001;
	double ALPHA = ( (2 * cutoff * _delT) / (2 + cutoff*_delT));
	output = ALPHA * ( (input_present - input_prev)/_delT) + (1 - ALPHA) * output_prev;
	//cout<<"input_present:"<<input_present<<endl<<"input_prev:"<<input_prev<<endl;
}

double amplitude = 30;
void generate_trajectory(double &qdes, double &qdotdes, double &qdotdotdes, double &motion_time){
	double omega = 2 * PI * f;
	qdes = amplitude * cos(omega* motion_time);
	qdotdes = -amplitude * omega * sin(omega * motion_time);
	qdotdotdes = -amplitude * omega * omega * cos(omega * motion_time);
}
double motion_time = 0;
bool initflag = true;
bool flag_datalogging = true;
int compute() {
    if (demo_mode == DEMO_MODE_TORQUE) {
		if (system_ready){
			if(flag_datalogging){
				_dataLogger.activate();
				flag_datalogging = false;
			}
		}

        for (int i = 0; i < NUM_AXIS; ++i) {
            if (system_ready) {
            	_time += period;
				//1. Assume that actual position and velocity (q, qdot) unit is changed to radian and radian/sec: DONE
				//2. Assume that actual torque unit is changed to Nm //DONE
				//3. generate desired trajectory (qdes, qdotdes, qdotdotdes) in radian, radian/sec, and radian/sec^2
				//4. Implement PID control for torque control\
				//4.1: error: qdes - q, qdotdes - qdot
				//4.2: PD: Kp * error - Kd*qdot
				//4.3: PID: Kp * error + Ki * integral_error + Kd * error_dot
				//4.4: 

				if(initflag){
					ZeroPos[i] = ActualPos[i];
					if(ActualPos[0] != 0) initflag = false;					
				}

				// generate trajectory 
				generate_trajectory(qdes[i], qdotdes[i], qdotdotdes[i], motion_time);
                // PID control for position, output as torque
				control_signal = pid_control(qdes[i], q[i], qdot[i], qdotdes[i], computed_torque[i]);
				TargetTor[i] = physical2ethercat(control_signal);
				motion_time += period;
            } else {
                _systemInterface_EtherCAT_EthercatCore.setServoOn(i);
				control_signal = 0;
                TargetTor[i] = 0;  // Zero torque when not ready
            }
        }
    } else if (demo_mode == DEMO_MODE_POSITION) {
        for (int i = 0; i < NUM_AXIS; ++i) {
            if (system_ready) {
                // Simple sinusoidal trajectory for position
                qdes[i] = 50000 * sin(PI2 * 0.2 * gt);
                TargetPos[i] = (INT32)(qdes[i]) + ZeroPos[i];
            } else {
                _systemInterface_EtherCAT_EthercatCore.setServoOn(i);
                TargetPos[i] = ZeroPos[i] = ActualPos[i];
            }
        }
    }
    return 0;
}
//EthercatCore_task	
void EthercatCore_run(void *arg)
{
	RTIME now, previous;
	
	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	_systemInterface_EtherCAT_EthercatCore.syncEcatMaster();
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (run)
	{
		//test
		//test2
		rt_task_wait_period(NULL); 	//wait for next cycle
		previous = rt_timer_read();

		/// TO DO: read data from sensors in EtherCAT system interface
		_systemInterface_EtherCAT_EthercatCore.processTxDomain();
		_systemInterface_EtherCAT_EthercatCore.readBuffer(0x60410, StatusWord);
		_systemInterface_EtherCAT_EthercatCore.readBuffer(0x60640, ActualPos);
		_systemInterface_EtherCAT_EthercatCore.readBuffer(0x606c0, ActualVel);
		_systemInterface_EtherCAT_EthercatCore.readBuffer(0x60770, ActualTor);
		_systemInterface_EtherCAT_EthercatCore.readBuffer(0x60610, ModeOfOperationDisplay);
		

		ethercat2physical();
		/// TO DO: Main computation routine...
		compute();
		
		_dataLogger.updateLoggedData(_time, q, qdot, core_tor);
		// Triggering logger saving
		_logCnt--;
		if (_logCnt <= 0)
		{
			_dataLogger.triggerSaving();
			_logCnt = LOG_DATA_SAVE_PERIOD; // 10s
		}

		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x607a0, TargetPos);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60ff0, TargetVel);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60710, TargetTor);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60600, ModeOfOperation);
		/*
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60400, Controlword);		
		*/
		_systemInterface_EtherCAT_EthercatCore.processRxDomain();

		if (system_ready)
			saveLogData();
			
		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (_systemInterface_EtherCAT_EthercatCore.isSystemReady())
		{
			system_ready=1;	//all drives have been done

			gt+= period;
			
			if (worst_time<ethercat_time) worst_time=ethercat_time;
			if(ethercat_time > max_time)
				++fault_count;
		}
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState=0;
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e8);
	
	while (1)
	{
		if (++count==15)
		{
			++stick;
			count=0;
		}
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;
			rt_printf("Time=%d.%d s, ", itime/1000, itime % 1000);
			rt_printf("dt= %li, worst= %li\n", ethercat_time, worst_time);
			rt_printf("test_print\n");
			
			if (_systemInterface_EtherCAT_EthercatCore.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
			else
				rt_printf("Master: Offline\n");

			if (_systemInterface_EtherCAT_EthercatCore.getRxDomainStatus())
				rt_printf("RxDomain: Online\n");
			else
				rt_printf("RxDomain: Offline\n");

			if (_systemInterface_EtherCAT_EthercatCore.getTxDomainStatus())
				rt_printf("TxDomain: Online\n");
			else
				rt_printf("TxDomain: Offline\n");
				
			for(i=0; i<NUM_AXIS; ++i){
				if (_systemInterface_EtherCAT_EthercatCore.getAxisEcatStatus(i, slaveState))
					rt_printf("\e[32;1mSlave: Online %i,  \e[0m\n", slaveState);
				else
					rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");
										
				rt_printf("\e[32;1m\t StatusWord: 0x%x,  \e[0m\n",		StatusWord[i]);
				rt_printf("\e[32;1m\t ModeOfOpDisp: 0x%x,  \e[0m\n",	ModeOfOperationDisplay[i]);

				

				rt_printf("\e[32;1m\t q: %f,  \e[0m\n", 	 	q[i]);
				rt_printf("\e[32;1m\t qdes: %f,  \e[0m\n", 	 	qdes[i]);
				rt_printf("\e[32;1m\t computed_torque: %f,  \e[0m\n", 	 	computed_torque[i]);
				rt_printf("\e[32;1m\t ActualTor: %i,  \e[0m\n", 	 	ActualTor[i]);
				rt_printf("\e[32;1m\t TargetPos: %i,  \e[0m\n", 	 	TargetPos[i]);
				rt_printf("\e[32;1m\t TargetVel: %i,  \e[0m\n", 	 	TargetVel[i]);
				rt_printf("\e[32;1m\t TargetTor: %i,  \e[0m\n", 	 	TargetTor[i]);
				rt_printf("\e[32;1m\t error: %f,  \e[0m\n", 	 	error);
				rt_printf("\e[32;1m\t error_dot: %f,  \e[0m\n", 	 	error_dot);
				

			}
			rt_printf("\n");
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}

		rt_task_wait_period(NULL); //wait for next cycle
	}
}

// GUI Command process cycle
void gui_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7);	// period = 1 (msec)

	INT8 modeOpDisp[NUM_AXIS] = {0};
	UINT16 status[NUM_AXIS] = {0};
	INT32 actPos[NUM_AXIS] = {0};
	INT32 actVel[NUM_AXIS] = {0};
	INT16 actTor[NUM_AXIS] = {0};

	INT8 modeOp[NUM_AXIS] = {0};
	float tarval[NUM_AXIS] = {0};
	float maxvel[NUM_AXIS] = {0};
	float maxacc[NUM_AXIS] = {0};
	float maxjerk[NUM_AXIS] = {0};
	
	while (1)
	{		
		if (guicontrolsocket.hasConnection())
		{
			for (int i=0; i<NUM_AXIS; i++)
			{
				modeOpDisp[i] = ModeOfOperationDisplay[i];
				status[i] = StatusWord[i];
				actVel[i] = ActualVel[i];
				actTor[i] = ActualTor[i];
				actPos[i] = ActualPos[i];			
			}

			guicontrolsocket.sendMotionData(modeOpDisp, status, actPos, actVel, actTor);

			if (guicontrolsocket.getMotionData(modeOp, tarval, maxvel, maxacc, maxjerk) != 0)
			{
				for (int index=0; index<NUM_AXIS; index++)
				{
					ModeOfOperation[index] = modeOp[index];
					
					switch (modeOp[index])
					{
					case OP_MODE_CYCLIC_SYNC_POSITION:
						Axis[index].setTrajBoundaryCond((double) maxvel[index], (double) maxacc[index]);
						Axis[index].setTarPosInCnt(tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_VELOCITY:
						Axis[index].resetTraj();
						Axis[index].setTarVelInCnt((INT32) tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_TORQUE:
						Axis[index].resetTraj();
						Axis[index].setTarTorInCnt((INT16) tarval[index]);
						break;
						
					default:
						Axis[index].setDataOut((INT32) tarval[index]);
						break;
					}
				}
			}
		}

		rt_task_wait_period(NULL);
	}
}

void plot_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100 ms)
	 */
	//rt_task_set_periodic(NULL, TM_NOW, 1e8);	// period = 100 (msec)

	while (1)
	{
		/// TO DO: You have to prepare data for NRMKDataSocket
		if (datasocket.hasConnection() && system_ready)
		{
			if (frontIdx < rearIdx)
			{
				datasocket.updateControlData(_loggingBuff[frontIdx].ActualPos, _loggingBuff[frontIdx].ActualVel);
				datasocket.update(_loggingBuff[frontIdx].Time);

				frontIdx++;
			}
			else if (rearIdx == MAX_BUFF_SIZE)
			{
				frontIdx = rearIdx = 0;
			}
		}
		else
		{
			frontIdx = rearIdx = 0;
		}
		sleep(1);
		//rt_task_wait_period(NULL);
	}
}

/****************************************************************************/
void signal_handler(int signum = 0)
{
	_dataLogger.deactivate();
	rt_task_delete(&plot_task);
	rt_task_delete(&gui_task);
	rt_task_delete(&EthercatCore_task);
	rt_task_delete(&print_task);
    printf("Servo drives Stopped!\n");

    _systemInterface_EtherCAT_EthercatCore.deinit();
    exit(1);
}

/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
	reset_fault(0);

	rt_print_auto_init(1);
	
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	if (argc>1)
	{
		amplitude = atof(argv[1]);
	}
	if (argc>2)
	{
		kp = atof(argv[2]);
	}
		if (argc>3)
	{
		kd = atof(argv[3]);
	}


	// double kp=1, ki=0.1, kd=0.1;
	
	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns=1000000; // nanosecond
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
	
	// Set the demo mode for EtherCAT application
	demo_mode = DEMO_MODE_TORQUE;
	if (demo_mode == DEMO_MODE_TORQUE)
	{
		// For CST (cyclic synchronous torque) control
		if (_systemInterface_EtherCAT_EthercatCore.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;
	}
	else if (demo_mode == DEMO_MODE_POSITION)
	{
		// For CSP (cyclic synchronous position) control
		if (_systemInterface_EtherCAT_EthercatCore.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_POSITION;
	}
	else // DEMO_MODE_GUI 
	{
		// For CSP (cyclic synchronous position) control
		if (_systemInterface_EtherCAT_EthercatCore.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_POSITION;		
	}
	
	// For trajectory interpolation
	initAxes();

	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
	
	// TO DO: Create control socket server
	if (guicontrolsocket.startServer(SOCK_TCP, 6868))
		printf("Control server started at IP of : %s on Port: %d\n", guicontrolsocket.getAddress(), 6868);

	printf("Waiting for Control Tool to connect...\n");
	guicontrolsocket.waitForConnection(0);

	// EthercatCore_task: create and start
	printf("Now running rt task ...\n");

	rt_task_create(&EthercatCore_task, "EthercatCore_task", 0, 99, 0);
	rt_task_start(&EthercatCore_task, &EthercatCore_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 80, 0);
	rt_task_start(&print_task, &print_run, NULL);
	
	// plotting: data socket comm
	rt_task_create(&plot_task, "plotting", 0, 80, 0);
	rt_task_start(&plot_task, &plot_run, NULL);

	// controlling: control socket
	rt_task_create(&gui_task, "gui_controlling", 0, 85, 0);
	rt_task_start(&gui_task, &gui_run, NULL);
	
	// Must pause here
	//pause();
	while (1)
	{
		
		
		
		
		usleep(1e5);
	}

	// Finalize
	signal_handler();

    return 0;
}



