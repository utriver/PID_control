/*
 * AgingDataLogger.h
 *
 *  Created on: 2019. 4. 15.
 *      Author: neuromeka
 */

#ifndef AGINGDATALOGGER_AGINGDATALOGGER_H_
#define AGINGDATALOGGER_AGINGDATALOGGER_H_

#pragma once

#include "../COREsys.h"
#include <time.h>
#include <string.h>		// string function definitions (strncpy)
#include <stdio.h>		// (FILE)

//#define NUM_AXIS (20)
#define AVG_BUFF_SIZE   6*60*24*4     		// 4 days (every 10 sec)
#define RAW_BUFF_SIZE	4000*60          //
#define MOVE_BUFF_SIZE 	4000*1           // sec
#define CTRL_BUFF_SIZE  30*4000			//4000*5
#define BUFF_HEAD_CUT 	0.1         		// 1.5sec after rest
#define BUFF_TAIL_CUT 	0.13   		      	// 0.5sec before rest
#define SKIP_IDX  4000*BUFF_HEAD_CUT

class AgingDataLogger
{
private:
	enum
		{
			CORE_NUM_AXIS = NUM_AXIS
		};

public:
	AgingDataLogger();
	~AgingDataLogger();

public:
	void calc_avg_value(bool isReset);
	void update_rt_buffer(RobotControlData ctrlData);
	void update_avg_buffer(RobotControlData ctrlData, int traj_phase);
	void write_rt_buffer();
	void write_avg_buffer();
	//void write_rt_buffer(char *core_num_product);
	//void write_avg_buffer(char *core_num_product);
	void attach_data_nametag(int argc, char **argv);
	bool set_logging_path(char *rt_logging_path, char *avg_logging_path);
public:
	//LOGGING_PACK _loggingBuff_raw[RAW_BUFF_SIZE];
	//LOGGING_PACK_DOUBLE _avgLogger_CW[AVG_BUFF_SIZE];
	//LOGGING_PACK_DOUBLE _avgLogger_CCW[AVG_BUFF_SIZE];
	//LOGGING_PACK_DOUBLE _loggingBuff_CCW[MOVE_BUFF_SIZE];
	//LOGGING_PACK_DOUBLE _loggingBuff_CW[MOVE_BUFF_SIZE];
	//LOGGING_PACK_CONTROLLER _loggingBuff_ctrl[CTRL_BUFF_SIZE];

	RobotControlData _loggingBuff_raw[RAW_BUFF_SIZE];
	RobotControlData _avgLogger_CW[AVG_BUFF_SIZE];
	RobotControlData _avgLogger_CCW[AVG_BUFF_SIZE];
	RobotControlData _loggingBuff_CW[MOVE_BUFF_SIZE];
	RobotControlData _loggingBuff_CCW[MOVE_BUFF_SIZE];
	RobotControlData _loggingBuff_ctrl[CTRL_BUFF_SIZE];
public:
	bool isAvgCW;
	bool isCalcAvg;
	bool CALC_AVERAGE_FRICTION = false;
	bool isRTbufferFilled;
	unsigned int logger_idx;
	int logger_idx_max;
	unsigned int ctrloggerIdx;
	unsigned int cwIdx;
	unsigned int ccwIdx;
	unsigned int log_idx;
	char *filename_cw;
	char *filename_ccw;
	char *filename;
	char *core_num_product;
	char *avg_data_logging_path;
	char *rt_data_logging_path;

};

#endif /* AGINGDATALOGGER_AGINGDATALOGGER_H_ */
