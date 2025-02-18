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
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

//
#include <cmath>
#include <cstdio>
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
#include "FrictionDataLogger/FrictionDataLogger.h"
#include "FrictionDatalogger/AgingDataLogger.h"
#include "CoreController/Controllers.h"
#include "COREsys.h"


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
double despos[NUM_AXIS] = {0,};
double qdot[NUM_AXIS] = {0,};
double desvel[NUM_AXIS] = {0,};
double qddot[NUM_AXIS] = {0,};
double desacc[NUM_AXIS] = {0,};
double core_tor[NUM_AXIS] = {0,};

/// TO DO: This is user-code.
double sine_amp=50000, f=0.25, period;
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
double friction_torque[NUM_AXIS] = {0};

INT32 	TargetPos[NUM_AXIS] = {0};
INT32 	TargetVel[NUM_AXIS] = {0};
INT16 	TargetTor[NUM_AXIS] = {0};
UINT32 	DataOut[NUM_AXIS] = {0};
UINT8 	ModeOfOperation[NUM_AXIS] = {0};
UINT16	Controlword[NUM_AXIS] = {0};

bool SAVE_MOVE_BUFFER = false;
bool WRITE_MOVE_BUFFER = false;

DataLogger _dataLogger;
unsigned int _logCnt;
double _time;
#define LOG_DATA_SAVE_PERIOD  180*4000

///// SDO Access /////////



//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

//trajectory class
typedef Controllers::JointMat JointMat;
typedef Controllers::JointVec JointVec;


Controllers _trajectory;
Controllers _PIDctr;
Controllers _filters;

FrictionDataLogger _frictionDataLogger;
AgingDataLogger _agingDataLogger;
RobotControlData ctrlData;

double qdes_lspb_init = 0;

JointVec _qdot_prev;
JointVec _qddot_prev;

/****************************************************************************/

// Xenomai RT tasks
RT_TASK EthercatCore_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK gui_task;
RT_TASK save_task;

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
		
			q[i] = (double) (ActualPos[i] - ZeroPos[i])*360/7929856*deg2rad	;
			qdot[i] = (double) ActualVel[i]*360/7929856*deg2rad;
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
double maxtorq = 150;
double physical2ethercat(double control_signal)
{
	for (int i=0; i<NUM_AXIS; ++i)
	{
		double temp_tau, temp_qdot;
		if (control_signal > maxtorq ) control_signal = maxtorq;
		else if (control_signal < -maxtorq) control_signal = -maxtorq;
		
		temp_tau = control_signal * Nm2Cnt;
		return (INT16) temp_tau;
		break;
	}
}
void stribeck_friction(double qdotdes, double &friction_torque){
	double Fc = 12.9389;
	double Fs = 14.2153;
	double Vs = 0.0002;
	double delta_s = 2.0000;
	double s2 = 42.9204;
	friction_torque = Fc + (Fs - Fc) * exp(-(qdotdes / Vs) * (qdotdes / Vs)) + s2*qdotdes;
}
double error_integral = 0;
double error, error_dot;
void arctan_friction(double qdotdes, double &friction_torque){
	double t_s = 21.4256;
	double t_sc = 0.81609;
	double t_v = 55.3774;
	double t_nlv = -20.9042;
	double Kv = 111;
	double delt = 9;
	double v = qdotdes;
	friction_torque = t_s*(2/PI)*atan(Kv*v)+t_sc*(2/PI)*atan(v*delt)+t_v*v+t_nlv*v*v*(2/PI)*atan(v*Kv);
	
}
double dz=0;
double g_v=0;
double z_=0;
void Lugre_friction(double qdotdes, double &friction_torque)
{
	double Fc = 12.9389;
	double Fs = 14.2153;
	double Vs = 0.0002;
    double s0 = 10.536233;
    double s1 = 2169.214629;
   	double s2 = 42.9204; 

    // 留덉같 怨꾩닔 怨꾩궛
    double g_v = Fc + (Fs - Fc) * exp(-(qdotdes / Vs) * (qdotdes / Vs));

    // dz 怨꾩궛 (�떆媛� 媛꾧꺽 dt瑜� 怨좊젮)
    double dt = 0.00025;  // 硫붿씤 肄붾뱶�� �룞�씪�븳 �떆媛� 媛꾧꺽
    dz = qdotdes - (s0 * abs(qdotdes) * z_) / g_v;
    z_ = z_ + dz * dt;  // �쟻遺꾩쓣 �쐞�빐 dt瑜� 怨깊븿
    friction_torque = s0*z_+s1*dz+s2*qdotdes;
}
    

int gms_counter=0;
#define N_MAXWELL 8
double z[N_MAXWELL] = {0};
// void GMS_friction(double qdotdes, double &friction_torque)
// {
//     // �븿�닔 �떆�옉�떆 friction_torque 珥덇린�솕
//     friction_torque = 0;

//     int N = 8;
//     double k_opt[8] = {143921.11506902, 39798223.8440106, 9363611.67828457, 67095.0889746429, 2558.20756818028, 27105750.4476824, 582985.183877143, 21357.4720391935};
//     double alpha_opt[8] = {0.277393295043814, 0.00185094082631222, 0.00365359131352696, 0.0100222860136948, 0.000445794147046117, 0.00290450020743669, 0.165052094736506, 0.100926088108215};
//     double C_opt = 27.5392; // 留덉같 吏��뿰 �긽�닔 //�쟾泥� �겕湲곌� 而ㅼ쭚
//     double vs_opt =0.95987; // �뒪�듃由щ깹 �냽�룄
//     double sigma_opt = 453.9787; // �젏�꽦 怨꾩닔 //怨≪꽑�쓽 理쒕�媛믪씠 而ㅼ쭚
//     double Fs = 21.4256; // �젙�쟻 留덉같�젰
//     double Fc = 21.4256 - 0.81609; // 荑⑤” 留덉같�젰
//     double dt = 0.00025;
//     double s_v = Fc+(Fs-Fc)*exp(-pow(qdotdes/vs_opt,2));

//     for(int n = 0; n < N; n++) {
//         // �뒳�씪�씠�뵫 �긽�깭 諛� �뒪�떛�궧 �긽�깭 泥섎━
//         if(gms_counter == 0) {
//             z[n] = 0;
//         }
//         if(fabs(z[n]) < alpha_opt[n] * s_v/k_opt[n] && fabs(qdotdes) < 1e-4) {
//             // �뒪�떛�궧 �긽�깭
//             dz = qdotdes;
// 			z[n] = z[n] + dz * dt;
// 			friction_torque = friction_torque + k_opt[n] * z[n] + alpha_opt[n]*dz;
//     }
//         }
//         else {
//             // �뒳�씪�씠�뵫 �긽�깭
//             dz = (C_opt * alpha_opt[n] / k_opt[n]) * ((qdotdes > 0 ? 1 : -1) - z[n] / (alpha_opt[n]*s_v/k_opt[n]));
//         }
//         // Maxwell �슂�냼 �긽�깭 蹂��닔 �뾽�뜲�씠�듃
        
       
        
        
//     // �젏�꽦 留덉같�젰 異붽�
//     friction_torque += sigma_opt * qdotdes;
// }
//  k = [66303.2909367738 66269.1682674476 66269.1063012516 2218793.36418749 2218793.32895251 317144.651565354 76718.3549064896 79123.4139371207 72558.6517033946 26765.7477754477 69870.0545341115 69838.745372707] ;      % �뒪�봽留� 媛뺤꽦 怨꾩닔;
//     alpha = [1.06492775941585 0.336883063963703 0.311386740179736 0.0209920389271211 0.0209920407778812 0.000255451067173599 0.00544452340345758 0.0130509874191778 0.00293665997264368 0.000255451092472014 0.000255450936089651 0.000255450906153214] ; % Maxwell �슂�냼 媛�以묒튂;
//     C = 9.3593 ;     % 留덉같 吏��뿰 �긽�닔;
//     Fs = 7.1638 ;    % �젙�쟻 留덉같�젰 (怨좎젙媛�);
//     Fc = 363.4886 ;     % 荑⑤” 留덉같�젰 (怨좎젙媛�);
//     vs = 7.3514 ;     % �뒪�듃由щ깹 �냽�룄;
//     sigma = 38.4811 ; % �젏�꽦 怨꾩닔;

void GMS_friction(double qdotdes, double &friction_torque) {
    // �븿�닔 �떆�옉 �떆 friction_torque 珥덇린�솕
    friction_torque = 0;

    int N = 12;
    // double k_opt[12] = {27918.2099063506, 27918.2114986032, 27918.2114893848, 27918.2124468587, 27918.2139948123, 27918.2156108475, 27918.2139913077, 27918.2124096552, 27918.212416005, 1141452.61311876, 28053.3165531205, 1303464.25814965};

	// double alpha_opt[12] = {0.0140792890679116, 0.0572528102734696, 0.0572528066850993, 0.0140792889142508, 0.0572528077023147, 0.0140792885720547, 0.0140792898955194, 0.057252808397232, 0.0407344224689363, 0.0119868930504995, 0.854479101852907, 0.0119868930949691};
	// double C_opt = 10.9834;
	// double Fs = 14.2153;
	// double Fc = 12.9389;
	// double vs = 0.0002;
	// double sigma_opt = 42.9204;
	// double k_opt[12] = {10864.4981169525, 255946.337508348, 5358.61846628718, 8521.54330238144, 12226.1184337744, 5317.80984723001, 5482.69851411415, 5317.809847222, 5792.26304968909, 5317.809847222, 5327.21815007195, 5317.809847222};
    // double alpha_opt[12] = {0.0617891128805635, 0.275569529121443, 0.0253912332602835, 0.0641477183062394, 0.0226653264437337, 0.0199929711739063, 0.012487357230447, 0.281386687643029, 0.0720593102642443, 0.0309771546691873, 0.0146741386208741, 0.0303694403235407};
	//ouble C_opt =  9.3593;
	// double Fs = 14.2153;
	// double Fc = 12.9389;
	// double vs = 0.0002;
	// double sigma_opt = 42.9204;
	//double k_opt[12] = {10888.2725255115, 342731.64421358, 5714.58673959759, 8751.52680196208, 9548.11184497022, 3202.73110165654, 6119.42415663208, 3864.28688129092, 100835.040665468, 4940.56713878274, 4479.5852236283, 4746.98089674031};
	//double alpha_opt[12] = {0.0350572514687255, 0.276258619901774, 0.0699630908826632, 0.0889833734956643, 0.0233291676452308, 0.0325846904159466, 0.0253456347872363, 0.280531797892668, 0.0477475626555615, 0.0200208660414608, 0.0119052206767593, 0.0173292659500855};
	//ouble C_opt = 14546.4614;
	//double Fs = 14.2153;
	//double Fc = 12.9389;
	//double vs = 0.0002;
	//double sigma_opt = 42.9204;
	//stribeck,v=0.0008
	double k_opt[12] = {177622.928544251, 44660.6234869604, 408487.823480988, 2113.37547445184, 425610.933168765, 16783.7073679804, 1433294.23402738, 770582.341157389, 258003.323607, 1189689.73147036, 434197.962372017, 13125.0497364916};
	double alpha_opt[12] = {0.136873837388196, 0.0835560477105503, 0.00747718305579766, 0.126738956508169, 0.0545629481476721, 0.177542791292639, 0.018442480552487, 0.0153304490558945, 0.00594541318346043, 0.00280037487231405, 0.0038631523629545, 0.0534922051009204};
	double C_opt = 8351.7184;
	double Fs = 15;
	double Fc = 14.4387;
	double vs = 0.0003;
	double sigma_opt = 38.5725;

    double dt = 0.00025; // �떆媛� 媛꾧꺽
	double t_s = 21.4256;
	double t_sc = 0.81609;
	double t_v = 100.3774;
	double t_nlv = -20.9042;
	double Kv = 111;
	double delt = 9;
	double v = qdotdes;
	double s_v= t_s+t_sc*atan(v*delt);
	double f_v= t_v*v+t_nlv*v*v*atan(v*Kv);
    // Stribeck 留덉같�젰 怨꾩궛
    // double s_v = Fc + (Fs - Fc) * exp(-pow(qdotdes / vs, 2));


    

    for (int n = 0; n < N; n++) {
        double dz = 0.0; // �긽�깭 蹂��솕�웾 珥덇린�솕

		// ���냽 �쁺�뿭: �뒪�떛�궧 諛� �봽由ъ뒳�씪�씠�뵫
		if (fabs(z[n]) < alpha_opt[n] * s_v / k_opt[n] && fabs(qdotdes) < 1e-3*5) {
			dz = qdotdes;  // �뒪�떛�궧 �긽�깭�뿉�꽌�뒗 �냽�룄�뿉 �쓽�빐 �긽�깭媛� 蹂��븿
		} else {
			dz = (C_opt * alpha_opt[n] / k_opt[n]) * ((qdotdes > 0 ? 1 : -1) - z[n] / (alpha_opt[n] * s_v / k_opt[n]));
		}
        

        // Maxwell �슂�냼 �긽�깭 蹂��닔 �뾽�뜲�씠�듃
        z[n] = z[n] + dz * dt;

        // 留덉같 �넗�겕 怨꾩궛
        friction_torque += k_opt[n] * z[n] + alpha_opt[n] * dz;
    }

    // �젏�꽦 留덉같�젰 異붽� (怨좎냽 �쁺�뿭�뿉�꽌�룄 �쟻�슜)
    // double viscous_friction = sigma_opt * qdotdes;
	friction_torque += f_v;

}

double kp =  2500;
double kd = 550;
double ki = 1500;
void pid_control_friction(double qdes, double q, double qdot, double qdotdes, double &computed_torque, double friction_torque) {
    
    error = qdes - q;
	error_dot = qdotdes - qdot;


//	computed_torque = kp * error - kd * qdot;
	computed_torque = 3.5*(kp * error + kd * error_dot + ki * error_integral);
//	error_integral += 0.001 * error; //1kHz
	error_integral += 0.00025 * error;
//	return computed_torque;
	//feedforward
	computed_torque +=friction_torque;
}

void pid_control(double qdes, double q, double qdot, double qdotdes, double &computed_torque) {
    error = qdes - q;
	error_dot = qdotdes - qdot;

//	computed_torque = kp * error - kd * qdot;
	computed_torque = 3.5*(kp * error + kd * error_dot + ki * error_integral);
//	error_integral += 0.001 * error; //1kHz
	error_integral += 0.00025 * error;
//	return computed_torque;
}


void LowPassDerivative(const double & input_prev, const double& input_present, const double& output_prev, const double& cutoff , double& output){
//	double _delT=0.001;
	double _delT=0.00025;
	double ALPHA = ( (2 * cutoff * _delT) / (2 + cutoff*_delT));
	output = ALPHA * ( (input_present - input_prev)/_delT) + (1 - ALPHA) * output_prev;
	//cout<<"input_present:"<<input_present<<endl<<"input_prev:"<<input_prev<<endl;
}



double amplitude=2 ;
int cycle_count = 0;
const int NUM_STEPS = 6;
const double MIN_AMP = 0.001;
const double MAX_AMP = 0.002;
// void generate_sin_trajectory(double &qdes, double &qdotdes, double &qdotdotdes, double &motion_time){
// 	double omega = 2 * PI * f;
	
// 	// �븳 二쇨린媛� �걹�궗�뒗吏� �솗�씤
// 	if(motion_time >= (1.0/f)) {
// 		cycle_count++;
// 		motion_time = 0;
		
// 		// �옄�뿰吏��닔 �븿�닔 �삎�깭濡� amplitude 利앷�
// 		if(cycle_count < NUM_STEPS) {
// 			double t = (double)cycle_count / (NUM_STEPS - 1);
// 			amplitude = MIN_AMP + (MAX_AMP - MIN_AMP) * (exp(t) - 1) / (exp(1) - 1);
// 		}
// 	}

// 	qdes = amplitude * (1-cos(omega* motion_time));
// 	qdotdes = amplitude * omega * sin(omega * motion_time);
// 	qdotdotdes = amplitude * omega * omega * cos(omega * motion_time);
// }
void generate_sin_trajectory(double &qdes, double &qdotdes, double &qdotdotdes, double &motion_time){
	double omega = 2 * PI * f;
	qdes = amplitude * (1-cos(omega* motion_time) );
	qdotdes = amplitude * omega * sin(omega * motion_time);
	qdotdotdes = amplitude * omega * omega * cos(omega * motion_time);
}
void generate_trajectory(double &qdes, double &qdotdes, double &qdotdotdes,  bool gen){
	_trajectory.gen_lspb_trajectory(gen, qdes, qdotdes, qdotdotdes);
}

double motion_time = 0;
bool initflag = true;
bool flag_datalogging = true;
int isInitLspb = 1;
double q_init, qdes_lspb, qdes_prev;
int ctr_traj = 0;
bool gen = true;
double qprev = 0;
double last_increase_time = 0;
double MAX_AMPLITUDE = 10.0;  // 최대 amplitude 값 설정 
double MIN_AMPLITUDE = 5.0;
double motion_number;
double prev_cycle = 0;
bool first_cycle = true;
int dynamic_run = 1;
int compute() {
    if (demo_mode == DEMO_MODE_TORQUE) {
		if (system_ready)
		{	
			for(int i=0; i<NUM_AXIS; i++)
			{
				if(flag_datalogging)
				{
    					// Start of Selection
    					q_init = q[i];   // 첫 번째 실제 위치
					WRITE_MOVE_BUFFER = true;
					// _dataLogger.activate();
					flag_datalogging = false;
				}
			
				if(isInitLspb == 1)
				{
					qdes_lspb_init = -qdes[i];   //first desired position
					isInitLspb = 0;
				}

				if(ctr_traj < _trajectory.max_traj_size-1)
				{
						// Start of Selection
					qdes[i] = q_init + qdes[i] + qdes_lspb_init;
				}else{
					
					const double q_last = 0.0061047 ;
					gen = false;
					qdes[i] = q_last;
					ctr_traj = _trajectory.max_traj_size+1000000;
					
					// _dataLogger.deactivate();
				}	
				
					ctr_traj++; //=motion_time

					qdes_prev = qdes[i];
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

				// LowPassDerivative(qprev,q[i], qdot[i], fc, qdotdes[i]);
				// generate trajectory 
				// generate_trajectory(qdes[i], qdotdes[i], qdotdotdes[i], gen);
//////////////////////////////////static test///////////////////////////////////////
				// generate_sin_trajectory(qdes[i], qdotdes[i], qdotdotdes[i], motion_time);	
				// pid_control(qdes[i], q[i], qdot[i], qdotdes[i], computed_torque[i]);
				// control_signal = computed_torque[i];
////////////////////////////////////////////////////////////////////////////////////////////////////////
				// control_signal =(fc-amplitude)*sin(PI2*f*gt);
				// control_signal =amplitude;

				// 3주기마다 amplitude 증가코드
				// static double prev_cycle = 0;
				// if (gt >= (2.0/f) && gt - prev_cycle >= (2.0/f)) {
				// 	if (amplitude < MAX_AMPLITUDE) {
				// 		amplitude += 2.0;
				// 		prev_cycle = gt;
				// 		printf("Amplitude increased to: %f at time %f\n", amplitude, gt);
				// 		control_signal = amplitude*sin(PI2*f*fmod(gt, 2.0/f));

				// 	}else if(amplitude == MAX_AMPLITUDE){
				// 		control_signal = 0;
				// 	}

				// }
				 // 기본 사인파 신호 생성
 ////////////////////////////////////////////////////////////////////////////////////////////////////////               
                control_signal = amplitude * sin(PI2 * f * gt);  // 기본 사인파 신호 생성
                
                // 주기 체크 및 amplitude 증가 로직 
                if (gt >= (2.0/f)) {  // 첫 주기 이후부터 체크
                    double current_cycle = floor(gt * f / 2.0) * (2.0/f);
                    if (current_cycle > prev_cycle) {  // 새로운 주기 시작
                        if (amplitude < MAX_AMPLITUDE && dynamic_run==1) {
                            amplitude += 2.0;
                            printf("Amplitude increased to: %f at time %f\n", amplitude, gt);
                        } else if (amplitude == MAX_AMPLITUDE) {
                            amplitude = 0;  // MAX_AMPLITUDE 도달 시 정지
                            dynamic_run = 0;
                        }
                        prev_cycle = current_cycle;
                    }
                }
////////////////////////////////////////////////////////////////////////////////////////////////////////
				// GMS_friction(qdotdes[i], friction_torque[i]);
				// control_signal = friction_torque[i];
				TargetTor[i] = physical2ethercat(control_signal);
				// TargetTor[i] = physical2ethercat(control_signal);
				// qdes[i] = _trajectory.gen_lspb_trajectory(true);
				motion_time += period;
				qprev=q[i];
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


#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable:4996)


//extract constant velocity data from csv file
std::vector<std::string> split(const std::string& line, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

//1. read csv file
//2. find the 5th column value
//3. extract the data which is same the value I want
//4. mean each part of the data 
//5. save the 4th column data and 7th column data to the new csv file'


#define N_PARAMS 5    // �뙆�씪誘명꽣 媛쒖닔
#define N_DATA 100    // �뜲�씠�꽣 �깦�뵆 媛쒖닔
#define TOLERANCE 1e-6
#define MAX_ITERS 100


double percent_extract = 0;
int total_count = 0;  // �쟾泥� �뜲�씠�꽣 媛쒖닔瑜� ���옣�븷 蹂��닔
int current_count = 0;

#define MAX_LINE_LENGTH 2048  // �뜑 �겙 踰꾪띁 �겕湲� �궗�슜

void extract_data(double &percent_extract)

{
	// �젅�� 寃쎈줈 �궗�슜
	const char* base_path = "/home/user/release/";  // �떎�젣 �꽕移� 寃쎈줈濡� �닔�젙
	char filepath1[256], filepath2[256], filepath3[256];
	
	snprintf(filepath1, sizeof(filepath1), "%sfriction_data/RT_test/RT-data-1.csv", base_path);
	snprintf(filepath2, sizeof(filepath2), "%sfriction_data/RT_test/RT-data-2.csv", base_path);
	
	FILE *fp1 = fopen(filepath1, "r");
	FILE *fp2 = fopen(filepath2, "r");
	
	if (fp1 == NULL || fp2 == NULL) {
		printf("�뙆�씪�쓣 �뿴 �닔 �뾾�뒿�땲�떎. 寃쎈줈: %s, %s\n", filepath1, filepath2);
		return;
	}

	// �뙆�씪 �궡�슜 �뵒踰꾧퉭
	char line[MAX_LINE_LENGTH];
	int line_count = 0;
	printf("�뙆�씪 �궡�슜 �솗�씤 以�...\n");
	
	// 泥� 紐� 以� 異쒕젰�븯�뿬 �뜲�씠�꽣 �삎�떇 �솗�씤
	while (fgets(line, sizeof(line), fp1) && line_count < 5) {
		printf("Line %d: %s", line_count++, line);
	}
	rewind(fp1);  // �뙆�씪 �룷�씤�꽣瑜� �떎�떆 泥섏쓬�쑝濡�
	rewind(fp2);  // �뙆�씪 �룷�씤�꽣瑜� �떎�떆 泥섏쓬�쑝濡�`

	
	// Define target velocities
	std::vector<double> target_vels = {

    0.0030, 0.0119, 0.0212, 0.0311, 0.0414, 0.0522, 0.0636, 0.0756, 0.0882, 0.1014,

    0.1154, 0.1300, 0.1454, 0.1615, 0.1785, 0.1963, 0.2151, 0.2348, 0.2555, 0.2773,

    0.3002, 0.3243, 0.3496, 0.3761, 0.4041, 0.4334, 0.4643, 0.4967, 0.5308, 0.5667,

    0.6043, 0.6439, 0.6855, 0.7292, 0.7752, 0.8235, 0.8743, 0.9277, 0.9837, 1.0427,

    1.1047, 1.1698, 1.2382, 1.3102, 1.3858, 1.4653, 1.5488, 1.6366, 1.7289, 1.8259,

	-0.0030, -0.0119, -0.0212, -0.0311, -0.0414, -0.0522, -0.0636, -0.0756, -0.0882, -0.1014,

	-0.1154, -0.1300, -0.1454, -0.1615, -0.1785, -0.1963, -0.2151, -0.2348, -0.2555, -0.2773,

	-0.3002, -0.3243, -0.3496, -0.3761, -0.4041, -0.4334, -0.4643, -0.4967, -0.5308, -0.5667,

	-0.6043, -0.6439, -0.6855, -0.7292, -0.7752, -0.8235, -0.8743, -0.9277, -0.9837, -1.0427,

	-1.1047, -1.1698, -1.2382, -1.3102, -1.3858, -1.4653, -1.5488, -1.6366, -1.7289, -1.8259
	};



	// Create vector of vectors to store matching data for each target velocity
	std::vector<std::vector<std::vector<double>>> matching_data(target_vels.size());

	
	// Process file 1
	while (fgets(line, sizeof(line), fp1)) {
		if (ferror(fp1)) {
			printf("�뙆�씪 �씫湲� �삤瑜� 諛쒖깮\n");
			clearerr(fp1);
			continue;
		}
		
		std::string str_line(line);
		std::vector<std::string> tokens = split(str_line, ',');
		if (tokens.size() >= 7) {
			double vel = std::stod(tokens[4]);
			vel = round(vel * 10000.0) / 10000.0;  // �냼�닔�젏 4�옄由ш퉴吏� 諛섏삱由�
			
			// �뵒踰꾧퉭�쓣 �쐞�븳 異쒕젰 異붽�
			printf("Reading velocity: %.6f\n", vel);
			
			// �뜑 愿����븳 鍮꾧탳 諛⑹떇 �궗�슜
			for (size_t i = 0; i < target_vels.size(); i++) {
				// �젙�솗�븳 �씪移� ���떊 洹쇱궗媛� 鍮꾧탳
				if (fabs(vel - target_vels[i]) < 1e-5) {
					std::vector<double> row;
					row.push_back(std::stod(tokens[3]));
					row.push_back(std::stod(tokens[6]));
					matching_data[i].push_back(row);
					printf("Match found for velocity %.6f\n", target_vels[i]);
					break;
				}
			}
		}
	}

	// Process file 2 
	while (fgets(line, sizeof(line), fp2)) {
		std::string str_line(line);
		std::vector<std::string> tokens = split(str_line, ',');
		if (tokens.size() >= 7) {
			double vel = std::stod(tokens[4]);
			vel = round(vel * 10000.0) / 10000.0;  // �냼�닔�젏 4�옄由ш퉴吏� 諛섏삱由�
			// Check against each target velocity
			for (size_t i = 0; i < target_vels.size(); i++) {
				if (fabs(vel - target_vels[i]) < 1e-5) {
					std::vector<double> row;
					row.push_back(std::stod(tokens[3])); // 4th column
					row.push_back(std::stod(tokens[6])); // 7th column

					matching_data[i].push_back(row);
					printf("Match found for velocity %.6f\n", target_vels[i]);
					break;
				}
			}
		}
	}

	fclose(fp1);
	fclose(fp2);

	//mean each part of the data
	std::vector<double> mean_data_4th;    
	std::vector<double> mean_data_7th;    
	mean_data_4th.reserve(target_vels.size());
	mean_data_7th.reserve(target_vels.size());

	for (size_t i = 0; i < target_vels.size(); i++) {
		double sum_4th = 0;
		double sum_7th = 0;
		size_t valid_data_count = matching_data[i].size();
		
		// �뜲�씠�꽣媛� �뾾�뒗 寃쎌슦 泥섎━
		if (valid_data_count == 0) {
			mean_data_4th.push_back(0.0);  // �삉�뒗 �떎瑜� 湲곕낯媛�
			mean_data_7th.push_back(0.0);  // �삉�뒗 �떎瑜� 湲곕낯媛�
			printf("Warning: No data found for velocity %f\n", target_vels[i]);
			continue;
		}

		// �뜲�씠�꽣 �빀�궛 �떆 �쑀�슚�꽦 寃��궗
		for (size_t j = 0; j < valid_data_count; j++) {
			double val_4th = matching_data[i][j][0];
			double val_7th = matching_data[i][j][1];
			
			if (std::isfinite(val_4th)) {
				sum_4th += val_4th;
			} else {
				printf("Warning: Invalid value in 4th column for velocity %f\n", target_vels[i]);
				valid_data_count--;
			}
			
			if (std::isfinite(val_7th)) {
				sum_7th += val_7th;
			} else {
				printf("Warning: Invalid value in 7th column for velocity %f\n", target_vels[i]);
				valid_data_count--;
			}
		}

		// �쑀�슚�븳 �뜲�씠�꽣媛� �엳�뒗 寃쎌슦�뿉留� �룊洹� 怨꾩궛
		if (valid_data_count > 0) {
			mean_data_4th.push_back(sum_4th / valid_data_count);
			mean_data_7th.push_back(sum_7th / valid_data_count);
		} else {
			mean_data_4th.push_back(0.0);  // �삉�뒗 �떎瑜� 湲곕낯媛�
			mean_data_7th.push_back(0.0);  // �삉�뒗 �떎瑜� 湲곕낯媛�
			printf("Warning: No valid data for velocity %f\n", target_vels[i]);
		}
	}
	
	//뙆씪 옣 떆 쑀슚꽦 寃궗 異붽
	FILE *fp3 = fopen("friction_data/RT_test/RT-data-3.csv", "w");
	if (fp3 == NULL) {
		printf("Error: Cannot open output file\n");
		return;
	}
	int extract = 0;
	for (size_t i = 0; i < target_vels.size(); i++) {
		if (std::isfinite(mean_data_4th[i]) && std::isfinite(mean_data_7th[i])) {
			fprintf(fp3, "%f,%f\n", mean_data_4th[i], mean_data_7th[i]);
		} else {
			fprintf(fp3, "0.0,0.0\n");  // �삉�뒗 �떎瑜� 湲곕낯媛�
			printf("Warning: Invalid mean values for velocity %f\n", target_vels[i]);
		}
		extract++;
		percent_extract = (double)extract / target_vels.size() * 100;
	}
	fclose(fp3);
}


int filenum1 = 1;
int filenum2 = 1;
double percent_ready = 0;
bool need_processing = false;
// void static_calculation(void *arg)
// {

// 	while(run)
//     {
//         if (need_processing) {
//             printf("Data extraction completed: %.2f%%\n", percent_extract);

//             // Python 스크립트 실행
//             const char* python_cmd = "python3 /home/user/release/friction_Id/static_calculate.py";
//             int result = system(python_cmd);
            
//             if (result == 0) {
//                 printf("Static calculation completed successfully\n");
// 				dynamic_id = 1;
//             } else {
//                 printf("Error executing static calculation script\n");
//             }
//             need_processing = false;
//         }
//     }
	
// }
// void dynamic_calculation(void *arg)
// {

// 	while(run)
//     {
//         if (need_processing) {

//             // Python 스크립트 실행
//             const char* python_cmd = "python3 /home/user/release/friction_Id/gms_calculate_1.py";
//             int result = system(python_cmd);
            
//             if (result == 0) {
//                 printf("Dynamic calculation completed successfully\n");
// 				dynamic_id = 0;
//             } else {
//                 printf("Error executing dynamic calculation script\n");
//             }
//             need_processing = false;
//         }
//     }
	
// }
// RT 태스크는 최소한의 작업만 수행

void save_run(void *arg) 
{
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

    while(run)
    {
        if (SAVE_MOVE_BUFFER)
        {
           	    _frictionDataLogger.write_rt_buffer(filenum2, percent_ready);
                SAVE_MOVE_BUFFER = false;
                WRITE_MOVE_BUFFER = false;
				printf("save_run\n");
				extract_data(percent_extract);

            
        }
        rt_task_wait_period(NULL);
    }
}


//EthercatCore_task	
void EthercatCore_run(void *arg)
{
	unsigned int runcount=0;
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
		runcount++;
		previous = rt_timer_read();
		//position trajectory
		 // CSV �뙆�씪�뱾�씠 �엳�뒗 �뵒�젆�넗由� 寃쎈줈
		
		
		
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


		// if(system_ready)
		// {
    	// 		// Start of Selection
    	// 		_dataLogger.updateLoggedData(_time, q, qdot, core_tor, qdes, qdotdes);
		// 			// Triggering logger saving
		// 	_logCnt--;
		// 	if (_logCnt <= 0)
		// 	{
		// 		_dataLogger.triggerSaving();
		// 		_logCnt = LOG_DATA_SAVE_PERIOD; // 10s
		// 	}
		// }

		ctrlData.time = gt;
		for (int i = 0; i < NUM_AXIS; i++)
		{
			ctrlData.q[i] = q[i];
			//ctrlData.ActualPOS[i] = 0;
			//ctrlData.ActualVel[i] = 0;
			ctrlData.qdes[i] = qdes[i];
			ctrlData.qdot[i] = qdot[i];
			ctrlData.qdotdes[i] = qdotdes[i];
//			ctrlData.qdotdes[i] = qddot[i];
			ctrlData.friction_torque[i] = friction_torque[i];
			ctrlData.coretor[i] = core_tor[i];  //Nm
			ctrlData.TargetTor[i] = TargetTor[i]; //Ethercat
			//ctrlData.ActualTor[i] = ActualTor[i];
			//ctrlData.sensortor[i] =  0 ;
//			ctrlData.sensortemperature[i] = mech_tor/5;
			//ctrlData.sensortemperature[i] = 0;
		}
		if(system_ready)
		{
			if(WRITE_MOVE_BUFFER){
					_frictionDataLogger.update_rt_buffer(ctrlData);				

			
//				printf("******* START writing to the buffer *****\n");

				if (_frictionDataLogger.isRTbufferFilled)
				{
//					printf("*******Buffer is full. Start saving data *****\n");
					WRITE_MOVE_BUFFER = false;
					SAVE_MOVE_BUFFER = true;
					_frictionDataLogger.isRTbufferFilled = false;
				}
			}
		}
		
		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x607a0, TargetPos);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60ff0, TargetVel);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60710, TargetTor);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60600, ModeOfOperation);
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60650, friction_torque);
		/*
		_systemInterface_EtherCAT_EthercatCore.writeBuffer(0x60400, Controlword);		
		*/
		_systemInterface_EtherCAT_EthercatCore.processRxDomain();
		// double senstor = m8010_target_torque;
		
		if (system_ready)
			saveLogData();
			
		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (_systemInterface_EtherCAT_EthercatCore.isSystemReady()&& (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)))
		{
			
			// system_ready=1;	//all drives have been done

                system_ready = 1;
				gt += period;	

			// gt+= period;
			
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
				rt_printf("\e[32;1m\t qdot: %f,  \e[0m\n", 	 	qdot[i]);
				rt_printf("\e[32;1m\t computed_torque: %f,  \e[0m\n", 	 	computed_torque[i]);
				rt_printf("\e[32;1m\t ActualTor: %i,  \e[0m\n", 	 	ActualTor[i]);
				rt_printf("\e[32;1m\t TargetPos: %i,  \e[0m\n", 	 	TargetPos[i]);
				rt_printf("\e[32;1m\t TargetVel: %i,  \e[0m\n", 	 	TargetVel[i]);
				rt_printf("\e[32;1m\t TargetTor: %i,  \e[0m\n", 	 	TargetTor[i]);
				rt_printf("\e[32;1m\t error: %f,  \e[0m\n", 	 	error);
				rt_printf("\e[32;1m\t error_dot: %f,  \e[0m\n", 	 	error_dot);
				rt_printf("\e[32;1m\t friction_torque: %f,  \e[0m\n", 	 	friction_torque[i]);
				rt_printf("\e[32;1m\t percent_ready: %f,  \e[0m\n", 	 	percent_ready);
				rt_printf("\e[32;1m\t percent_extract: %f,  \e[0m\n", 	 	percent_extract);
				

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
	// _dataLogger.deactivate();
	rt_task_delete(&save_task);
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

		motion_number = atof(argv[1]);
	}
	if (argc>2)
	{
		filenum2 = atof(argv[2]);
		// ��吏곸엫 踰덊샇 �꽕�젙
		//0: not apply
		//1: stribeck
		//2: arctan
		//3: lugre
		//4: GMS
	}
	if (argc>3)
	{
		amplitude = atof(argv[3]);
	}

	if (argc>4)
	{
		kp=atof(argv[4]);
	}
	if (argc>5)	
	{
		kd=atof(argv[5]);
	}
	if (argc>6)
	{
		ki=atof(argv[6]);
	}



	// double kp=1, ki=0.1, kd=0.1;
	
	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
//	cycle_ns=1000000; // nanosecond 1kHz
	cycle_ns=250000; // nanosecond 4kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit/
	// period = 0.00025;
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
	//confirm the trajectory file is loaded
	char traj2[] = "/home/user/release/friction_data/RT_test/";
	char traj3[] = "/home/user/release/friction_data/Avg_test/";
	if(!_frictionDataLogger.set_logging_path(traj2,traj3))

	{
		printf("Set Data Logger Path\n");
		exit(1);
	}
	if (!_trajectory.read_trajectory("/home/user/release/data_csvFile/")) {
			printf("Set Data Logger Path\n");
			exit(1);
		}

	// printf("trajectory file is loaded\n");
	// std::cout << "Trajectory Positions:" << std::endl;
    // for (size_t i = 0; i < _trajectory._qdes_traj_pos.size(); ++i) {
    //     std::cout << "Position [" << i << "]: " << _trajectory._qdes_traj_pos[i] << std::endl;
    // }

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

	// save: create and start
	rt_task_create(&save_task, "saving", 0, 90, 0); //int rt_task_create(RT_TASK *task, const char *name, int stksize, 슦꽑닚쐞, int mode);
	rt_task_start(&save_task, &save_run, NULL);
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



