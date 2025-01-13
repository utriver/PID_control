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

//EtherCAT Master ******************************************************************
#include "ecrt.h"

#include "SystemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.h"
#include "EcatDataSocket.h"
#include "EcatControlSocket.h"

#include "ServoAxis.h"

//-user defined-///////////////////////////////////////////////////////////////////
#include "CoreController/Controllers.h"
#include "COREsys.h"
#include "DataLogger/DataLogger.h"



#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME		(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

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
NRMKHelper::SystemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor _systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> controlsocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;

int flag_init_pos = 0;

int InitFlag[NUM_AXIS + NUM_TOOLS] = {0,0};

// EtherCAT Data (in pulse)
INT32 	ZeroPos[NUM_AXIS + NUM_TOOLS] = {0,0};
INT32 	ActualPos[NUM_AXIS + NUM_TOOLS] = {0,0};
INT32 	ActualVel[NUM_AXIS + NUM_TOOLS] = {0,0};
INT16 	ActualTor[NUM_AXIS + NUM_TOOLS] = {0,0};
UINT32	DataIn[NUM_AXIS + NUM_TOOLS] = {0,0};
union{
	UINT32	GrossValue[NUM_AXIS + NUM_TOOLS] = {0,0};
	float   Realtorque[NUM_AXIS + NUM_TOOLS];
}torquedata;

INT8	Modesofoperationdisplay[NUM_AXIS + NUM_TOOLS] = {0,0};

INT32 	TargetPos[NUM_AXIS + NUM_TOOLS] = {0,0};
INT32 	TargetVel[NUM_AXIS + NUM_TOOLS] = {0,0};
INT16 	TargetTor[NUM_AXIS + NUM_TOOLS] = {0,0};
UINT32 	DataOut[NUM_AXIS + NUM_TOOLS] = {0,0};


///// SDO Access /////////



//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

//////////////////////////////////////////////////////// Nm2cnt conversion
double Nm2CntConv[NUM_AXIS] = {0};

double _kp=0.0, _kd = 0.0, _ki = 0.0;
// mode = 0 sinusoidal target torque
// mode = 1  step target torque with PID controller
unsigned int  mode = 0;


typedef Controllers::JointMat JointMat;
typedef Controllers::JointVec JointVec;


Controllers _PIDctr;
Controllers _trajectories;
DataLogger _dataLogger;
unsigned int _logCnt;
double _time;


/****************************************************************************/

// Xenomai RT tasks
RT_TASK NeuromekaNRMKDriveWithTorqueSensor_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK control_task;

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
	/*
	const int gearRatio[NUM_AXIS] = {0,0};
	const int pulsePerRevolution[NUM_AXIS] = {0,0};
	const double ratedTau[NUM_AXIS] = {0,0};
	const int dirQ[NUM_AXIS] = {0,0};
	const int dirTau[NUM_AXIS] = {0,0};
	const int zeroPos[NUM_AXIS] = {0,0};
	*/

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
		
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.setServoOn(i, true);
	}
	
	return 1;
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

/****************************************************************************/
double q, qdot, qdes, q_init, qdes_lspb, qdes_prev;
double coretor;
double tau_pid = 0.0, _tau_c = 0.0;
int targettorTemp = 0;
unsigned int cnt = 0;
int isInitLspb = 1;
double qdes_lspb_init = 0;
unsigned int cnt_trajectory = 0;

int compute()
{
	// Logging data in real-time
	_time += 0.001;

	if (system_ready)
	{
		// desired position trajectory
		if (flag_init_pos == 0){
			q_init = q;
			_dataLogger.activate();
			flag_init_pos = 1;
		}

		qdes_lspb = _trajectories.gen_lspb_trajectory(true);
		if(isInitLspb == 1)
		{
			qdes_lspb_init = -qdes_lspb;
			isInitLspb = 0;
		}
		if(cnt_trajectory < _trajectories.max_traj_size)
		{
			qdes = q_init + qdes_lspb + qdes_lspb_init;
		}
		else
		{
			qdes = qdes_prev;
			cnt_trajectory = _trajectories.max_traj_size + 100;
		}
		cnt_trajectory++;

		_PIDctr.PIDcontroller2(_kp, _kd, _ki, q, qdes, tau_pid);

		// torque limit
		if(tau_pid > 500) tau_pid = 500;
		else if(tau_pid < -500) tau_pid = -500;

		qdes_prev = qdes;
	}
	
	// CST
	for (int i=0; i<NUM_AXIS; ++i)
	{
		if (system_ready){
#if(0)
			// sinusoidal target torque
			TargetTor[i] = (int) _tau_c*Nm2CntConv[i];
#else
			// step target torque with PID controller
			TargetTor[i] = (int)  tau_pid*Nm2CntConv[i];
#endif
		}

		else
			TargetTor[i]=0;
	}

	
	return 0;
}

// Neuromeka_NRMK_Drive_task
double deg2rad = PI/180;
// NeuromekaNRMKDriveWithTorqueSensor_task	
void NeuromekaNRMKDriveWithTorqueSensor_run(void *arg)
{
	unsigned int runcount=0;
	RTIME now, previous;
	
	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.syncEcatMaster();
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (run)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		runcount++;

		if (!run)
		{
			break;
		}

		previous = rt_timer_read();

		/// TO DO: read data from sensors in EtherCAT system interface
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.processTxDomain();
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(COE_POSITION, ActualPos);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(COE_VELOCITY, ActualVel);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(COE_TORQUE, ActualTor);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(DATA_IN, DataIn);
#if defined (_USE_ENDTOOL_BOARD)
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(0x44f04, torquedata.GrossValue);
#endif
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.readBuffer(0x60610, Modesofoperationdisplay);		
		
		if (!system_ready) initAxes();

		q  = (double)  (ActualPos[0]*360)/EncoderResolution;// (double) ((ActualPos[0] - ZeroPos[0])/EncoderResolution)*PI2;
		q *= deg2rad;
		qdot = (double) (ActualVel[0]*360)/EncoderResolution; //(double) (ActualVel[0]/EncoderResolution)*PI2;
		qdot *= deg2rad;
		Nm2CntConv[0] = 48.0 / (0.0884 * 121);
		coretor = (double) ActualTor[0];
		coretor /= Nm2CntConv[0];

		/// TO DO: Main computation routine...
		compute();
		
		double _q[NUM_AXIS], _qdot[NUM_AXIS], _qdes[NUM_AXIS];
		for(int i=0; i <NUM_AXIS; i++){
			_q[i] = q;
			_qdot[i] = qdot;
			_qdes[i] = qdes;
		}
		_dataLogger.updateLoggedData(_time, _q, _qdot, _qdes);
//		_dataLogger.updateLoggedData(_time, q, qdot, qdes);

		// Triggering logger saving
		_logCnt--;
		if (_logCnt <= 0)
		{
			_dataLogger.triggerSaving();
			_logCnt = LOG_DATA_SAVE_PERIOD; // 10s
		}

		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.writeBuffer(COE_POSITION, TargetPos);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.writeBuffer(COE_VELOCITY, TargetVel);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.writeBuffer(COE_TORQUE, TargetTor);
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.writeBuffer(DATA_OUT, DataOut);
		/*
		
		*/
		_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.processRxDomain();
			
		if (system_ready)
			saveLogData();
			
		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.isSystemReady() && (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) )
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
		if (++count==10)
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
			
			if (_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
			else
				rt_printf("Master: Offline\n");

			if (_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getRxDomainStatus())
				rt_printf("RxDomain: Online\n");
			else
				rt_printf("RxDomain: Offline\n");

			if (_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getTxDomainStatus())
				rt_printf("TxDomain: Online\n");
			else
				rt_printf("TxDomain: Offline\n");
				
			for(i=0; i<NUM_AXIS; ++i){
				if (_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getAxisEcatStatus(i, slaveState))
					rt_printf("\e[32;1mSlave: Online %i,  \e[0m\n", slaveState);
				else
					rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");
					
				rt_printf("\e[32;1m Status: 0x%x,  \e[0m\n", 	_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getAxisCoEStatus(i));
				rt_printf("\e[32;1m\t ActPos: %i,  \e[0m\n", 	 	ActualPos[i]);
				rt_printf("\e[32;1m\t ActTor: %i,  \e[0m\n", 	 	ActualTor[i]);
				rt_printf("\e[32;1m\t TargetTor: %i,  \e[0m\n", 	 	TargetTor[i]);
#if defined (_USE_ENDTOOL_BOARD)
				rt_printf("\e[32;1m\t GrossValue: %.6f,  \e[0m\n", 	 	torquedata.Realtorque[i + NUM_TOOLS]);
#endif
				/*rt_printf("\e[32;1m\t Modesofoperationdisplay: %i,  \e[0m\n", 	 	Modesofoperationdisplay[i]);
				*/
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

// Command process cycle
void control_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7);	// period = 1 (msec)

#if defined (_USE_ENDTOOL_BOARD)
	INT8 modeOpDisp[NUM_AXIS + NUM_TOOLS] = {0,0};
	UINT16 status[NUM_AXIS + NUM_TOOLS] = {0,0};
	INT32 actPos[NUM_AXIS + NUM_TOOLS] = {0,0};
	INT32 actVel[NUM_AXIS + NUM_TOOLS] = {0,0};
	INT16 actTor[NUM_AXIS + NUM_TOOLS] = {0,0};

	INT8 modeOp[NUM_AXIS + NUM_TOOLS] = {0,0};
	float tarval[NUM_AXIS + NUM_TOOLS] = {0,0};
	float maxvel[NUM_AXIS + NUM_TOOLS] = {0,0};
	float maxacc[NUM_AXIS + NUM_TOOLS] = {0,0};
	float maxjerk[NUM_AXIS + NUM_TOOLS] = {0,0};
#else
	INT8 modeOpDisp[NUM_AXIS ] = {0};
	UINT16 status[NUM_AXIS ] = {0};
	INT32 actPos[NUM_AXIS ] = {0};
	INT32 actVel[NUM_AXIS ] = {0};
	INT16 actTor[NUM_AXIS ] = {0};

	INT8 modeOp[NUM_AXIS ] = {0};
	float tarval[NUM_AXIS ] = {0};
	float maxvel[NUM_AXIS ] = {0};
	float maxacc[NUM_AXIS ] = {0};
	float maxjerk[NUM_AXIS ] = {0};
#endif
	
	while (1)
	{		
		if (controlsocket.hasConnection())
		{
#if defined (_USE_ENDTOOL_BOARD)
			for (int i=0; i<NUM_AXIS + NUM_TOOLS; i++)
#else
			for (int i=0; i<NUM_AXIS; i++)
#endif
			{
				modeOpDisp[i] = _systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getModeOperation(i);
				status[i] = _systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.getAxisCoEStatus(i);
				actVel[i] = ActualVel[i];
				actTor[i] = ActualTor[i];
				actPos[i] = ActualPos[i];			
			}

			controlsocket.sendMotionData(modeOpDisp, status, actPos, actVel, actTor);

			if (controlsocket.getMotionData(modeOp, tarval, maxvel, maxacc, maxjerk) != 0)
			{
#if defined (_USE_ENDTOOL_BOARD)
				for (int index=0; index<NUM_AXIS + NUM_TOOLS; index++)
#else
				for (int index=0; index<NUM_AXIS; index++)
#endif
				{
					_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.setModeOperation(index, modeOp[index]);
					
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
		usleep(1000);
		//rt_task_wait_period(NULL);
	}
}

/****************************************************************************/
void signal_handler(int signum = 0)
{
	_dataLogger.deactivate();
	rt_task_delete(&plot_task);
	rt_task_delete(&control_task);
	rt_task_delete(&NeuromekaNRMKDriveWithTorqueSensor_task);
	rt_task_delete(&print_task);
    printf("Servo drives Stopped!\n");

    _systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.deinit();
    exit(1);
}

/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns=1000000; // nanosecond
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
	
#if 0
	if(argc>1)
		sine_amp=atof(argv[1]);
	if(sine_amp>1000) sine_amp=1000;

	if(argc>2)
		_kp = atof(argv[2]);
	if(argc>3)
		_kd = atof(argv[3]);
	if(argc>4)
		_ki = atof(argv[4]);
	if(argc>5)
		mode = atoi(argv[5]);
#else
	if(argc>1)
		_kp = atof(argv[1]);
	if(argc>2)
		_kd = atof(argv[2]);
	if(argc>3)
		_ki = atof(argv[3]);

#endif

	if(_kp < 0) _kp = 0.0;
	if(_kd < 0) _kd = 0.0;
	if(_ki < 0) _ki = 0.0;
	if(mode < 0 ) mode = 0;

	// Set the operation mode for EtherCAT servo drives
	
	// For CST (cyclic synchronous torque) control
	_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns);
	
	// For CSP (cyclic synchronous position) control
//	_systemInterface_EtherCAT_NeuromekaNRMKDriveWithTorqueSensor.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns);
	
	// For trajectory interpolation
	initAxes();
	char traj[] = "Trajectory/";
	if(!_trajectories.read_trajectory(traj))
	{
		exit(1);

	}


	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
	
	// TO DO: Create control socket server
	if (controlsocket.startServer(SOCK_TCP, 6868))
		printf("Control server started at IP of : %s on Port: %d\n", controlsocket.getAddress(), 6868);

	printf("Waiting for Control Tool to connect...\n");
	controlsocket.waitForConnection(0);

	// NeuromekaNRMKDriveWithTorqueSensor_task: create and start
	printf("Now running rt task ...\n");

	rt_task_create(&NeuromekaNRMKDriveWithTorqueSensor_task, "NeuromekaNRMKDriveWithTorqueSensor_task", 0, 99, 0);
	rt_task_start(&NeuromekaNRMKDriveWithTorqueSensor_task, &NeuromekaNRMKDriveWithTorqueSensor_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 80, 0);
	rt_task_start(&print_task, &print_run, NULL);
	
	// plotting: data socket comm
	rt_task_create(&plot_task, "plotting", 0, 80, 0);
	rt_task_start(&plot_task, &plot_run, NULL);

	// controlling: control socket
	rt_task_create(&control_task, "controlling", 0, 85, 0);
	rt_task_start(&control_task, &control_run, NULL);
	
	// Must pause here
	//pause();
	while (1)
	{
		
		
		
		
		usleep(10000);
	}

	// Finalize
	signal_handler();

    return 0;
}


