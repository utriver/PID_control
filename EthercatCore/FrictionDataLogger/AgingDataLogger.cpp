/*
 * AgingDataLogger.cpp
 *
 *  Created on: 2019. 4. 15.
 *      Author: neuromeka
 */

#include "AgingDataLogger.h"

AgingDataLogger::AgingDataLogger()
: ctrloggerIdx(0)
, isAvgCW(false)
, isRTbufferFilled(false)
, isCalcAvg(false)
, logger_idx(0)
, logger_idx_max(0)
, cwIdx(0)
, ccwIdx(0)
, log_idx(0)
{
	filename_cw = new char[100];
	filename_ccw = new char[100];
	filename = new char[100];
	core_num_product = new char[100];
	rt_data_logging_path = new char[100];
	avg_data_logging_path = new char[100];

}

AgingDataLogger::~AgingDataLogger()
{
	delete filename_cw;
	delete filename_ccw;
	delete filename;
	delete core_num_product;
	delete rt_data_logging_path;
	delete avg_data_logging_path;
}

void AgingDataLogger::attach_data_nametag(int argc, char **argv)
{
	if (argc >= 3)
	{
		core_num_product[0] = 0;
		int i =0;
		for (i = 0; argv[2][i] != 0; i++)
		{
			core_num_product[i] = argv[2][i];
		}
		core_num_product[i] = 0;
	}else
	{
		sprintf(core_num_product,"%s", "CORE_SET_T");
	}
}

void AgingDataLogger::calc_avg_value(bool isReset)
{
	//LOGGING_PACK_DOUBLE sum_value;
	RobotControlData sum_value;


	if (isReset)
	{

		logger_idx = 0;
	}

	if (isCalcAvg)
	{
		printf("are you alive###############################\n");
		for (int axis=0; axis<CORE_NUM_AXIS;axis++)
		{
			//sum_value.ActualPos[axis] = 0;
			//sum_value.ActualVel[axis] = 0;
			sum_value.q[axis] = 0;
			sum_value.qdot[axis] = 0;
			sum_value.ActualTor[axis] = 0;
			sum_value.TargetTor[axis] = 0;
			sum_value.coretor[axis] = 0;
		}

		for (int i=0; i<MOVE_BUFF_SIZE; i++)
		{
			for (int axis=0; axis<CORE_NUM_AXIS; axis++)
			{
				if (isAvgCW)
				{
					//sum_value.ActualPos[axis] += _loggingBuff_CW[i].ActualPos[axis];
					//sum_value.ActualVel[axis] += _loggingBuff_CW[i].ActualVel[axis];
					sum_value.q[axis] += _loggingBuff_CW[i].q[axis];
					sum_value.qdot[axis] += _loggingBuff_CW[i].qdot[axis];
					sum_value.ActualTor[axis] += _loggingBuff_CW[i].ActualTor[axis];
					sum_value.TargetTor[axis] += _loggingBuff_CW[i].TargetTor[axis];
					sum_value.coretor[axis] += _loggingBuff_CW[i].coretor[axis];
//					printf("CW: = %f\n", sum_value.coretor[axis]);
				}
				else
				{
					//sum_value.ActualPos[axis] += _loggingBuff_CCW[i].ActualPos[axis];
					//sum_value.ActualVel[axis] += _loggingBuff_CCW[i].ActualVel[axis];
					sum_value.q[axis] += _loggingBuff_CCW[i].q[axis];
					sum_value.qdot[axis] += _loggingBuff_CCW[i].qdot[axis];
					sum_value.ActualTor[axis] += _loggingBuff_CCW[i].ActualTor[axis];
					sum_value.TargetTor[axis] += _loggingBuff_CCW[i].TargetTor[axis];
					sum_value.coretor[axis] += _loggingBuff_CCW[i].coretor[axis];
//					printf("CCW: = %f\n", sum_value.coretor[axis]);

				}
			}
		}
		// Save in Average logger buffer
		if (logger_idx < AVG_BUFF_SIZE)
		{
			time_t     now = time(0);
			struct tm  tstruct;
			//char       buf[80];
			tstruct = *localtime(&now);
			//strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

			if (isAvgCW)
				//strftime(_avgLogger_CW[logger_idx].Date, 50, "%Y-%m-%d.%X", &tstruct);
				strftime(_avgLogger_CW[logger_idx].date, 50, "%Y-%m-%d.%X", &tstruct);
			else
				//strftime(_avgLogger_CCW[logger_idx].Date, 50, "%Y-%m-%d.%X", &tstruct);
				strftime(_avgLogger_CCW[logger_idx].date, 50, "%Y-%m-%d.%X", &tstruct);

			for (int axis=0; axis<NUM_AXIS;axis++)
			{
				if (isAvgCW)
				{
					//_avgLogger_CW[logger_idx].ActualPos[axis] = sum_value.ActualPos[axis]/(MOVE_BUFF_SIZE);
					//_avgLogger_CW[logger_idx].ActualVel[axis] = sum_value.ActualVel[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CW[logger_idx].q[axis] = sum_value.q[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CW[logger_idx].qdot[axis] = sum_value.qdot[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CW[logger_idx].ActualTor[axis] = sum_value.ActualTor[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CW[logger_idx].TargetTor[axis] = sum_value.TargetTor[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CW[logger_idx].coretor[axis] = sum_value.coretor[axis]/(MOVE_BUFF_SIZE);
//					printf("_avgLogger_CW.coretor: = %f\n", _avgLogger_CW[logger_idx].coretor[axis] );
				}
				else
				{
					//_avgLogger_CCW[logger_idx].ActualPos[axis] = sum_value.ActualPos[axis]/(MOVE_BUFF_SIZE);
					//_avgLogger_CCW[logger_idx].ActualVel[axis] = sum_value.ActualVel[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CCW[logger_idx].q[axis] = sum_value.q[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CCW[logger_idx].qdot[axis] = sum_value.qdot[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CCW[logger_idx].ActualTor[axis] = sum_value.ActualTor[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CCW[logger_idx].TargetTor[axis] = sum_value.TargetTor[axis]/(MOVE_BUFF_SIZE);
					_avgLogger_CCW[logger_idx].coretor[axis] = sum_value.coretor[axis]/(MOVE_BUFF_SIZE);

				}
			}
			if (!isAvgCW)
			{
				logger_idx++;
				printf("logger_idx: %d",logger_idx);
			}
		}

		isCalcAvg = false;

	}

//	printf("logger_idx: = %f\n", logger_idx);

}

void AgingDataLogger::write_avg_buffer()
{
	// Reallocation
	RobotControlData  *_avgLogger_CW_save = new RobotControlData[AVG_BUFF_SIZE];
	RobotControlData  *_avgLogger_CCW_save = new RobotControlData[AVG_BUFF_SIZE];

	for (int i=0; i<logger_idx_max-1; i++)
	{
		strncpy(_avgLogger_CW_save[i].date, _avgLogger_CW[i].date, 50);
		strncpy(_avgLogger_CCW_save[i].date, _avgLogger_CCW[i].date, 50);
		for (int axis=0; axis<CORE_NUM_AXIS; axis++)
		{
			//_avgLogger_CW_save[i].ActualPos[axis] = _avgLogger_CW[i].ActualPos[axis];
			//_avgLogger_CW_save[i].ActualVel[axis] = _avgLogger_CW[i].ActualVel[axis];
			_avgLogger_CW_save[i].q[axis] = _avgLogger_CW[i].q[axis];
			_avgLogger_CW_save[i].qdot[axis] = _avgLogger_CW[i].qdot[axis];
			_avgLogger_CW_save[i].ActualTor[axis] = _avgLogger_CW[i].ActualTor[axis];
			_avgLogger_CW_save[i].TargetTor[axis] = _avgLogger_CW[i].TargetTor[axis];
			_avgLogger_CW_save[i].coretor[axis] = _avgLogger_CW[i].coretor[axis];
			_avgLogger_CW_save[i].StartTor[axis] = _avgLogger_CW[0].ActualTor[axis];
			_avgLogger_CW_save[i].SatTor[axis] = _avgLogger_CW[logger_idx_max-2].ActualTor[axis];



			//_avgLogger_CCW_save[i].ActualPos[axis] = _avgLogger_CCW[i].ActualPos[axis];
			//_avgLogger_CCW_save[i].ActualVel[axis] = _avgLogger_CCW[i].ActualVel[axis];
			_avgLogger_CCW_save[i].q[axis] = _avgLogger_CCW[i].q[axis];
			_avgLogger_CCW_save[i].qdot[axis] = _avgLogger_CCW[i].qdot[axis];
			_avgLogger_CCW_save[i].ActualTor[axis] = _avgLogger_CCW[i].ActualTor[axis];
			_avgLogger_CCW_save[i].TargetTor[axis] = _avgLogger_CCW[i].TargetTor[axis];
			_avgLogger_CCW_save[i].coretor[axis] = _avgLogger_CCW[i].coretor[axis];
			_avgLogger_CCW_save[i].StartTor[axis] = _avgLogger_CCW[0].ActualTor[axis];
			_avgLogger_CCW_save[i].SatTor[axis] = _avgLogger_CCW[logger_idx_max-2].ActualTor[axis];
		}
	}

	time_t     now = time(0);
	struct tm  tstruct;
	tstruct = *localtime(&now);
	char		avg_data_date[50];
	strftime(avg_data_date, 50, "%Y-%m-%d.%X", &tstruct);


	char info[50];
	char *info_addr;

	info[0] = 0;
	int i;

	for (i = 0; avg_data_date[i] != 0; i++) {
		info[i] = avg_data_date[i];
	}

	while (1){
		info_addr = strchr(info,'.');
		if (info_addr) *info_addr = '-';
		else
		{
			info_addr = strchr(info,':');
			if (info_addr) *info_addr = '-';
			else break;
		}
	}


	sprintf(filename_cw, "%sCW_%s%s%s.csv",avg_data_logging_path,core_num_product,"_",info);
	sprintf(filename_ccw, "%sCCW_%s%s%s.csv",avg_data_logging_path,core_num_product,"_",info);

	FILE *file_cw;
	FILE *file_ccw;

	file_cw = fopen(filename_cw,"w");
	file_ccw = fopen(filename_ccw,"w");

	for (int i=0; i<logger_idx_max-1; i++)
	{
		fprintf(file_cw,"%s, ",_avgLogger_CW_save[i].date);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%f, ", _avgLogger_CW_save[i].q[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%f, ", _avgLogger_CW_save[i].qdot[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%d, ", _avgLogger_CW_save[i].ActualTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%d, ", _avgLogger_CW_save[i].TargetTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%f, ", _avgLogger_CW_save[i].coretor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%d, ", _avgLogger_CW_save[i].StartTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_cw,"%d, ", _avgLogger_CW_save[i].SatTor[j]);
		fprintf(file_cw,"\n");

		fprintf(file_ccw,"%s, ",_avgLogger_CCW_save[i].date);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%f, ", _avgLogger_CCW_save[i].q[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%f, ", _avgLogger_CCW_save[i].qdot[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%d, ", _avgLogger_CCW_save[i].ActualTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%d, ", _avgLogger_CCW_save[i].TargetTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%f, ", _avgLogger_CCW_save[i].coretor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%d, ", _avgLogger_CCW_save[i].StartTor[j]);
		for (int j=0; j<CORE_NUM_AXIS; j++)
			fprintf(file_ccw,"%d, ", _avgLogger_CCW_save[i].SatTor[j]);
		fprintf(file_ccw,"\n");

	}
	fclose(file_cw);
	fclose(file_ccw);

	delete[] _avgLogger_CW_save;
	delete[] _avgLogger_CCW_save;

}

void AgingDataLogger::write_rt_buffer()
{
	RobotControlData *_rtLogger_save = new RobotControlData[CTRL_BUFF_SIZE];

	for (int i=0; i<CTRL_BUFF_SIZE; i++)
	{
        _rtLogger_save[i].time = _loggingBuff_ctrl[i].time;
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].q[j] = _loggingBuff_ctrl[i].q[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdes[j] = _loggingBuff_ctrl[i].qdes[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdot[j] = _loggingBuff_ctrl[i].qdot[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdotdes[j] = _loggingBuff_ctrl[i].qdotdes[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qddot[j] = _loggingBuff_ctrl[i].qddot[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].ActualPOS[j] = _loggingBuff_ctrl[i].ActualPOS[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].coretor[j] = _loggingBuff_ctrl[i].coretor[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].sensortor[j] = _loggingBuff_ctrl[i].sensortor[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].gravtor[j] = _loggingBuff_ctrl[i].gravtor[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].coretemperature[j] = _loggingBuff_ctrl[i].coretemperature[i];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].sensortemperature[j] = _loggingBuff_ctrl[i].sensortemperature[i];
	}

	time_t     now = time(0);
	struct tm  tstruct;
	tstruct = *localtime(&now);
	char		rt_data_date[50];
	strftime(rt_data_date, 50, "%Y-%m-%d.%X", &tstruct);

	//WYLEE-start
	char info[50];
	char *info_addr;

	info[0] = 0;
	int i;
	for (i = 0; rt_data_date[i] != 0; i++) {
		info[i] = rt_data_date[i];
	}

	while (1){
		info_addr = strchr(info,'.');
		if (info_addr) *info_addr = '-';
		else
		{
			info_addr = strchr(info,':');
			if (info_addr) *info_addr = '-';
			else break;
		}
	}

	sprintf(filename, "%sRT_%s%s%s.csv",rt_data_logging_path,core_num_product,"_",info);

	FILE *file;

	try{
		file = fopen(filename,"w");
		if (file == NULL)
		{
			throw ERROR_FILE_LOAD;
		}
	}catch (int exception)
	{
		switch(exception)
		{
		case 4:
			printf("Please make 'Logging' folder in this directory...\n");
			break;
		default:
			break;
		}
		exit(1);
	}

	for (int i=0; i<CTRL_BUFF_SIZE; i++)
	{
		fprintf(file,"%f, ", _rtLogger_save[i].time);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].q[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].qdes[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].qdot[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].qdotdes[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].qddot[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%i, ", _rtLogger_save[i].ActualPOS[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].coretor[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].sensortor[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].gravtor[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].coretemperature[j]);
		for (int j=0;j<CORE_NUM_AXIS;j++)
			fprintf(file,"%f, ", _rtLogger_save[i].sensortemperature[j]);
		fprintf(file,"\n");
	}

	fclose(file);
	delete[] _rtLogger_save;

}

void AgingDataLogger::update_avg_buffer(RobotControlData ctrlData, int traj_phase)
{
	// Friction data
	if (!isCalcAvg)
	{
		switch (traj_phase)
		{
		case 0:
			// Rest
			cwIdx = 0;
			ccwIdx = 0;
			log_idx = 0;
			isCalcAvg = false;
//			CALC_AVERAGE_FRICTION = false;

			break;
		case 1:
			// Forward (CW)
			// Skip: BUFF_HEAD_CUT*4000
			// End: MOVE_BUFF_SIZE
			if (log_idx > SKIP_IDX)
			{
				if (cwIdx < MOVE_BUFF_SIZE)
				{
					for (int i=0; i<CORE_NUM_AXIS; i++)
					{
						_loggingBuff_CW[cwIdx].q[i] = ctrlData.q[i];
						_loggingBuff_CW[cwIdx].qdot[i] = ctrlData.qdot[i];
						_loggingBuff_CW[cwIdx].ActualTor[i] = ctrlData.ActualTor[i];
						_loggingBuff_CW[cwIdx].TargetTor[i] = ctrlData.TargetTor[i];
						_loggingBuff_CW[cwIdx].coretor[i] = ctrlData.coretor[i];
					}
					cwIdx++;
				}
				else
				{
					// Flag for Averaging/Writing friction value
//					CALC_AVERAGE_FRICTION = true;
					printf("is it working #################################\n");
					isCalcAvg = true;
					isAvgCW = true;


					cwIdx = 0;
					log_idx = 0;
				}
			}
			else
			{
				log_idx++;
			}
			break;
		case 2:
			// Backward (CCW)
			if (log_idx > SKIP_IDX)
			{
				if (ccwIdx < MOVE_BUFF_SIZE)
				{
//					printf("check **********##### %i  #######**********\n", ccwIdx);
					for (int i=0; i<CORE_NUM_AXIS; i++)
					{
						_loggingBuff_CCW[ccwIdx].q[i] = ctrlData.q[i];
						_loggingBuff_CCW[ccwIdx].qdot[i] = ctrlData.qdot[i];
						_loggingBuff_CCW[ccwIdx].ActualTor[i] = ctrlData.ActualTor[i];
						_loggingBuff_CCW[ccwIdx].TargetTor[i] = ctrlData.TargetTor[i];
						_loggingBuff_CCW[ccwIdx].coretor[i] = ctrlData.coretor[i];
					}
					ccwIdx++;
				}
				else
				{
					// Flag for Averaging/Writing friction value
//					CALC_AVERAGE_FRICTION = true;
					isCalcAvg = true;
					isAvgCW = false;

					ccwIdx = 0;
					log_idx = 0;
				}
			}
			else
			{
				log_idx++;
			}

			break;
		default:
			break;
		}
	}
}

void AgingDataLogger::update_rt_buffer(RobotControlData ctrlData)
{
	if (ctrloggerIdx < CTRL_BUFF_SIZE)
	{
		_loggingBuff_ctrl[ctrloggerIdx].time = ctrlData.time;
		for (int i=0; i<CORE_NUM_AXIS; i++)
		{
			_loggingBuff_ctrl[ctrloggerIdx].q[i] = ctrlData.q[i];
			_loggingBuff_ctrl[ctrloggerIdx].qdes[i] = ctrlData.qdes[i];
			_loggingBuff_ctrl[ctrloggerIdx].qdot[i] = ctrlData.qdot[i];
			_loggingBuff_ctrl[ctrloggerIdx].qdotdes[i] = ctrlData.qdotdes[i];
			// _loggingBuff_ctrl[ctrloggerIdx].qddot[i] = ctrlData.qddot[i];
			// _loggingBuff_ctrl[ctrloggerIdx].ActualPOS[i] = ctrlData.ActualPOS[i];
			_loggingBuff_ctrl[ctrloggerIdx].coretor[i] = ctrlData.coretor[i];
			// _loggingBuff_ctrl[ctrloggerIdx].FricTor[i] = ctrlData.FricTor[i];
			// _loggingBuff_ctrl[ctrloggerIdx].sensortor[i] = ctrlData.sensortor[i];
			// _loggingBuff_ctrl[ctrloggerIdx].ActualTor[i] = ctrlData.ActualTor[i];
			// _loggingBuff_ctrl[ctrloggerIdx].ActualVel[i] = ctrlData.ActualVel[i];
			// _loggingBuff_ctrl[ctrloggerIdx].TargetTor[i] = ctrlData.TargetTor[i];
			// _loggingBuff_ctrl[ctrloggerIdx].coretemperature[i] = ctrlData.coretemperature[i];
			// _loggingBuff_ctrl[ctrloggerIdx].sensortemperature[i] = ctrlData.sensortemperature[i];
		}
		ctrloggerIdx++;
	}
	else
	{
		isRTbufferFilled = true;
		ctrloggerIdx = 0;
	}
}

bool AgingDataLogger::set_logging_path(char *rt_logging_path, char *avg_logging_path)
{
	sprintf(rt_data_logging_path,"%s",rt_logging_path);
	sprintf(avg_data_logging_path,"%s",avg_logging_path);

	return 1;
}

