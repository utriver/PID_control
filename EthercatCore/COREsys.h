/*
 * COREsys.h
 *
 *  Created on: Jan 3, 2022
 *      Author: Meseret
 */

#ifndef CORESYS_H_
#define CORESYS_H_

#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <native/task.h>
#include <string.h>		// string function definitions

#define _USE_ENDTOOL_BOARD

#define NUM_AXIS	(1)		//Modify this number to indicate the actual number of motor on the network
#if defined (_USE_ENDTOOL_BOARD)
#define NUM_TOOLS	(1)
#endif

#define ERROR_POS_FILE_NOT_FOUND 1
#define ERROR_VEL_FILE_NOT_FOUND 2
#define ERROR_ACC_FILE_NOT_FOUND 3
#define ERROR_FILE_LOAD	4

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

// Peitian motor
#define EncoderResolution 131072 //65536
//#define RatedTor_1	2.39 // [Nm]
#define RatedTor_1	0.67 // [Nm]
#define RatedTor_2	1.27 // [Nm]
#define RatedTor_3	0.67 // [Nm]
//#define TorqueConstant 0.0884
#define GearRatio 50
//#define ECAT2RAD PI2/EncoderResolution
//#define ECAT2NM500 4.4875 //48.0/(TorqueConstant*GearRatio)
//#define CORE500_TORQUE_LIMIT 450

#define deg2rad PI/180
#define rad2deg 180/PI

#define LOG_DATA_SAVE_PERIOD  200*4000


// ** control data configuration
struct RobotControlData
{
	double time;
	char date[50];
	double q[NUM_AXIS];
	double qdes[NUM_AXIS];
	double qdot[NUM_AXIS];
	double qdotdes[NUM_AXIS];
	double qddot[NUM_AXIS];
	double qddotdes[NUM_AXIS];
	double friction_torque[NUM_AXIS];
	double coretor[NUM_AXIS];
	double sensortor[NUM_AXIS];
	double gravtor[NUM_AXIS];
	double coretemperature[NUM_AXIS];
	double sensortemperature[NUM_AXIS];
	double inertia_estimation[NUM_AXIS];
	INT32 ActualPOS[NUM_AXIS];
	INT32 ActualVel[NUM_AXIS];
	INT32 ActualTor[NUM_AXIS];
	INT32 TargetTor[NUM_AXIS];
	INT32 StartTor[NUM_AXIS];
	INT32 SatTor[NUM_AXIS];
	//Meseret
	INT32 FricTor[NUM_AXIS];
	//Meseret end

};



#endif /* CORESYS_H_ */
