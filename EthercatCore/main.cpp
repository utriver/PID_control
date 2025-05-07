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

double error_integral = 0;
double error, error_dot;

double kp = 2500;
double kd = 200;
double ki = 3500;


void pid_control(double qdes, double q, double qdot, double qdotdes, double &computed_torque) {
    error = qdes - q;
	error_dot = qdotdes - qdot;

//	computed_torque = kp * error - kd * qdot;
	computed_torque = 3.5*(kp * error + kd * error_dot + ki * error_integral);
//	error_integral += 0.001 * error; //1kHz
	error_integral += 0.00025 * error;
//	return computed_torque;
}


double amplitude=2 ;


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
					
					const double q_last = 2.3964 ;
					gen = false;
					qdes[i] = q_last;
					ctr_traj = _trajectory.max_traj_size+1000000;

				}	
				
					ctr_traj++; //=motion_time

					qdes_prev = qdes[i];
			}
		}

        for (int i = 0; i < NUM_AXIS; ++i) {
            if (system_ready) {
            	_time += period;

				if(initflag){
					ZeroPos[i] = ActualPos[i];
					if(ActualPos[0] != 0) initflag = false;					
				}

 				generate_trajectory(qdes[i], qdotdes[i], qdotdotdes[i], gen);
				pid_control(qdes[i], q[i], qdot[i], qdotdes[i], computed_torque[i]);
				control_signal = computed_torque[i];

				TargetTor[i] = physical2ethercat(control_signal);

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
	const char* base_path = "/home/user/release/";
	char filepath[256], filepath1[256], filepath2[256], filepath3[256];
	
	snprintf(filepath, sizeof(filepath), "%sdata_csvFile/target_vel.csv", base_path);
	snprintf(filepath1, sizeof(filepath1), "%sfriction_data/RT_test/RT-data-1.csv", base_path);
	snprintf(filepath2, sizeof(filepath2), "%sfriction_data/RT_test/RT-data-2.csv", base_path);
	
	FILE *fp1 = fopen(filepath1, "r");
	FILE *fp2 = fopen(filepath2, "r");
	
	if (fp1 == NULL || fp2 == NULL) {
		printf("파일을 열 수 없습니다. 경로: %s, %s\n", filepath1, filepath2);
		return;
	}

	// 데이터 검증을 위해 파일의 첫 5줄을 화면에 출력하여 포맷 확인
	char line[MAX_LINE_LENGTH];
	int line_count = 0;
	
	while (fgets(line, sizeof(line), fp1) && line_count < 5) {
		printf("Line %d: %s", line_count++, line);
	}

	// 두 파일의 포인터를 파일의 시작으로 되돌려, 실제 데이터 처리 전 상태로 복귀
	rewind(fp1);
	rewind(fp2);

	std::vector<double> target_vels;
	std::ifstream file(filepath);
	if (!file.is_open()) std::cerr << "Error: cannot open file " << filepath << std::endl;
	
	std::string vel_line;
	while (std::getline(file, vel_line)) {
		std::istringstream linestream(vel_line);
		std::string token;
		while (std::getline(linestream, token, ',')) {
			try {
				double v = std::stod(token);
				target_vels.push_back(v);
			} catch (const std::invalid_argument&) {
				// 숫자로 변환할 수 없는 토큰은 무시
			}
		}
	}

	// target_vels와 fp1, fp2의 4, 7번째 열 값 쌍을 여러 개 저장할 수 있는 matching_data 2차원 벡터 배열 선언
	std::vector<std::vector<std::vector<double>>> matching_data(target_vels.size());

	
	// Process file 1
	// fgets: 한줄 씩 읽기
	while (fgets(line, sizeof(line), fp1)) {
		if (ferror(fp1)) { clearerr(fp1); continue; }
		auto tokens = split(std::string(line), ','); // 쉼표 기준 토큰 분리
		if (tokens.size() < 7) continue;

		double vel = std::stod(tokens[4]);
		vel = round(vel * 10000.0) / 10000.0; // 소수점 넷째 자리로 반올림
		
		printf("Reading velocity: %.6f\n", vel);
		for (size_t i = 0; i < target_vels.size(); i++) {
			if (fabs(vel - target_vels[i]) < 1e-5) {
				// 4번째(tokens[3])와 7번째(tokens[6]) 열 값 저장
				matching_data[i].push_back({
					std::stod(tokens[3]),
					std::stod(tokens[6])
				});
				printf("Match found for velocity %.6f\n", target_vels[i]);
				break;
			}
		}
	}

	// Process file 2 
	// fgets: 한줄 씩 읽기
	while (fgets(line, sizeof(line), fp2)) {
		if (ferror(fp2)) { clearerr(fp2); continue; }
		auto tokens = split(std::string(line), ','); // 쉼표 기준 토큰 분리
		if (tokens.size() < 7) continue;

		double vel = std::stod(tokens[4]);
		vel = round(vel * 10000.0) / 10000.0; // 소수점 넷째 자리로 반올림

		printf("Reading velocity: %.6f\n", vel);
		for (size_t i = 0; i < target_vels.size(); i++) {
			if (fabs(vel - target_vels[i]) < 1e-5) {
				// 4번째(tokens[3])와 7번째(tokens[6]) 열 값 저장
				matching_data[i].push_back({
					std::stod(tokens[3]),
					std::stod(tokens[6])
				});
				printf("Match found for velocity %.6f\n", target_vels[i]);
				break;
			}
		}
	}

	fclose(fp1);
	fclose(fp2);

	//mean each part of the data
	// 각 목표 속도마다 matching_data 크기 확인
	std::vector<double> mean_data_4th, mean_data_7th;
	mean_data_4th.reserve(target_vels.size());
	mean_data_7th.reserve(target_vels.size());

	for (size_t i = 0; i < target_vels.size(); i++) {
		double sum_4th = 0, sum_7th = 0;
		size_t valid_data_count = matching_data[i].size();
		
		// 데이터 없으면 0.0 저장, 경고 출력
		if (valid_data_count == 0) {
			mean_data_4th.push_back(0.0);
			mean_data_7th.push_back(0.0);
			printf("Warning: No data found for velocity %f\n", target_vels[i]);
			continue;
		}
		 // 유효성 검사하며 합산
		for (auto &row : matching_data[i]) {
			if (std::isfinite(row[0])) sum_4th += row[0];
			else { printf("Invalid 4th col at vel %f\n", target_vels[i]); valid_data_count--; }
			if (std::isfinite(row[1])) sum_7th += row[1];
			else { printf("Invalid 7th col at vel %f\n", target_vels[i]); valid_data_count--; }
		}

		if (valid_data_count > 0) {
			mean_data_4th.push_back(sum_4th / valid_data_count);
			mean_data_7th.push_back(sum_7th / valid_data_count);
		} else {
			mean_data_4th.push_back(0.0);
			mean_data_7th.push_back(0.0);
			printf("Warning: No valid data for velocity %f\n", target_vels[i]);
		}
	}
	
	FILE *fp3 = fopen("friction_data/RT_test/RT-data-3.csv", "w");
	if (!fp3) { printf("Error: Cannot open output file\n"); return; }

	int extract = 0;
	for (size_t i = 0; i < target_vels.size(); i++) {
		if (std::isfinite(mean_data_4th[i]) && std::isfinite(mean_data_7th[i])) {
			fprintf(fp3, "%f,%f\n", mean_data_4th[i], mean_data_7th[i]);
		} else {
			fprintf(fp3, "0.0,0.0\n");
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

		ctrlData.time = gt;
		for (int i = 0; i < NUM_AXIS; i++)
		{
			ctrlData.q[i] = q[i];
			ctrlData.qdes[i] = qdes[i];
			ctrlData.qdot[i] = qdot[i];
			ctrlData.qdotdes[i] = qdotdes[i];
			ctrlData.friction_torque[i] = friction_torque[i];
			ctrlData.coretor[i] = core_tor[i];  //Nm
			ctrlData.TargetTor[i] = TargetTor[i]; //Ethercat
		}
		if(system_ready)
		{
			if(WRITE_MOVE_BUFFER){
					_frictionDataLogger.update_rt_buffer(ctrlData);				

				if (_frictionDataLogger.isRTbufferFilled)
				{
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

	cycle_ns=250000; // nanosecond 4kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit/

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



