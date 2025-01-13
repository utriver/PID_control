/*
 * COREsys.h
 *
 *
 *  Created on: 2020. 10. 16.
 *      Author: MesAbay
 */
#ifndef CORESYS_H_
#define CORESYS_H_


#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <native/task.h>
#include <string.h>		// string function definitions

#define ERROR_POS_FILE_NOT_FOUND 1
#define ERROR_VEL_FILE_NOT_FOUND 2
#define ERROR_ACC_FILE_NOT_FOUND 3
#define ERROR_FILE_LOAD	4

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define Motor_enc_rel  131072  // encoder resolution (count per revolute)

// Conversion: Ecat to position [radian]
#define ECAT2RAD  PI2/Motor_enc_rel
#define RAD2ECAT Motor_enc_rel/PI2

// gear ratio
#define GEAR_RATIO  100

// Inertia: motor + HD
#define RotorInertia_withHD 2.5100e-05
#define INERTIA_J RotorInertia_withHD*GEAR_RATIO*GEAR_RATIO
#define ETHCAT2NM 0.0013

// conversion after transmission system
#define OUTER_POS_IN_RAD ECAT2RAD/GEAR_RATIO

#define INNER_POS_IN_ECAT RAD2ECAT*GEAR_RATIO

#define NUM_AXIS	(1)	

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;

#endif /* CORESYS_H_ */
