/*
 * FrictionDataLogger.cpp
 *
 *  Created on: Apr 15, 2022
 *      Author: meser
 */

#include "FrictionDataLogger.h"
// #define CTRL_BUFF_SIZE  180*4000

FrictionDataLogger::FrictionDataLogger()
:AgingDataLogger()
{
    core_id1 = new char [500];
    core_id2 = new char [500];
    logger_idx = this->logger_idx;
    logger_idx_max = this->logger_idx_max;
    log_idx = this->log_idx;
}

FrictionDataLogger::~FrictionDataLogger()
{
	delete core_id1;
	delete core_id2;
}


////////////////////////////////static mode////////////////////////////////
void FrictionDataLogger::write_rt_buffer(int filenum, double &percent_ready)
{

    RobotControlData *_rtLogger_save = new RobotControlData[ctrloggerIdx];

    for (int i = 0; i < ctrloggerIdx; i++)
    {
         _rtLogger_save[i].time = _loggingBuff_ctrl[i].time;
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].q[j] = _loggingBuff_ctrl[i].q[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdes[j] = _loggingBuff_ctrl[i].qdes[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdot[j] = _loggingBuff_ctrl[i].qdot[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].qdotdes[j] = _loggingBuff_ctrl[i].qdotdes[j];
        for (int j = 0; j < NUM_AXIS; j++)
			_rtLogger_save[i].ActualTor[j] = _loggingBuff_ctrl[i].ActualTor[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].TargetTor[j] = _loggingBuff_ctrl[i].TargetTor[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].coretor[j] = _loggingBuff_ctrl[i].coretor[j];
        for (int j = 0; j < NUM_AXIS; j++)
			_rtLogger_save[i].inertia_estimation[j] = _loggingBuff_ctrl[i].inertia_estimation[j];
    }

    time_t now = time(0);
    struct tm tstruct;
    tstruct = *localtime(&now);
    char rt_data_data[50];
    strftime(rt_data_data, 50, "%Y-%m-%d.%X", &tstruct);
    int ready = 0;
    char saving_time[50];
    for (int j = 0; j < NUM_AXIS; j++)
    {
        FILE *file1;
        sprintf(core_id1,"%sRT-data.csv", AgingDataLogger::rt_data_logging_path);
        file1 = fopen(core_id1,"w");
        for (int i = 0; i < ctrloggerIdx; i++)
        {
            fprintf(file1,"%f, ", _rtLogger_save[i].time); 			// 1 : A
            fprintf(file1,"%f, ", _rtLogger_save[i].q[j]); 			// 2 : B
            fprintf(file1,"%f, ", _rtLogger_save[i].qdes[j]); 		// 3 : C
            fprintf(file1,"%f, ", _rtLogger_save[i].qdot[j]); 		// 4 : D
            fprintf(file1,"%f, ", _rtLogger_save[i].qdotdes[j]); 	// 5 : E
            fprintf(file1,"%d, ", _rtLogger_save[i].ActualTor[j]); 	// 6 : F
            fprintf(file1,"%d, ", _rtLogger_save[i].TargetTor[j]); 	// 7 : G
            fprintf(file1,"%f, ", _rtLogger_save[i].coretor[j]); 	// 8 : H
            fprintf(file1,"%f, ", _rtLogger_save[i].inertia_estimation[j]); // 9 : I
            fprintf(file1,"\n");
            ++ready;

            percent_ready = (double)ready / (ctrloggerIdx*NUM_AXIS) * 100;
        }
        fclose(file1);
    }
    delete[] _rtLogger_save;
}
    
