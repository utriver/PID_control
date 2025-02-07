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
    core_id = new char [500];
    logger_idx = this->logger_idx;
    logger_idx_max = this->logger_idx_max;
    log_idx = this->log_idx;
}
FrictionDataLogger::~FrictionDataLogger()
{
	delete core_id;
}

void FrictionDataLogger::write_rt_buffer(int filenum, double &percent_ready)
{
    RobotControlData *_rtLogger_save = new RobotControlData[CTRL_BUFF_SIZE];


    for (int i = 0; i < CTRL_BUFF_SIZE; i++)
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
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].qddot[j] = _loggingBuff_ctrl[i].qddot[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].ActualVel[j] = _loggingBuff_ctrl[i].ActualVel[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].ActualPOS[j] = _loggingBuff_ctrl[i].ActualPOS[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].friction_torque[j] = _loggingBuff_ctrl[i].friction_torque[j];
        for (int j = 0; j < NUM_AXIS; j++)
            _rtLogger_save[i].coretor[j] = _loggingBuff_ctrl[i].coretor[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].sensortor[j] = _loggingBuff_ctrl[i].sensortor[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].ActualTor[j] = _loggingBuff_ctrl[i].ActualTor[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].TargetTor[j] = _loggingBuff_ctrl[i].TargetTor[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].coretemperature[j] = _loggingBuff_ctrl[i].coretemperature[j];
        // for (int j = 0; j < NUM_AXIS; j++)
        //     _rtLogger_save[i].sensortemperature[j] = _loggingBuff_ctrl[i].sensortemperature[j];
                  
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
        FILE *file;
        sprintf(core_id,"%sRT-data-%d.csv", AgingDataLogger::rt_data_logging_path, filenum);
        file = fopen(core_id,"w");
        for (int i = 0; i < CTRL_BUFF_SIZE; i++)
        {
            fprintf(file,"%f, ", _rtLogger_save[i].time); 			// 1 : A
            fprintf(file,"%f, ", _rtLogger_save[i].q[j]); 			// 2 : B
            fprintf(file,"%f, ", _rtLogger_save[i].qdes[j]); 		// 3 : C
            fprintf(file,"%f, ", _rtLogger_save[i].qdot[j]); 		// 4 : D
            fprintf(file,"%f, ", _rtLogger_save[i].qdotdes[j]); 	// 5 : E
            // fprintf(file,"%f, ", _rtLogger_save[i].qddot[j]); 		// 6 : F
            // fprintf(file,"%i, ", _rtLogger_save[i].ActualPOS[j]); 	// 7 : G
            fprintf(file,"%f, ", _rtLogger_save[i].friction_torque[j]); 	// 8 : H
            fprintf(file,"%f, ", _rtLogger_save[i].coretor[j]); 	// 8 : H
            // fprintf(file,"%f, ", _rtLogger_save[i].sensortor[j]); 	// 9 : I
            // fprintf(file,"%i, ", _rtLogger_save[i].ActualTor[j]); 	// 10 : J
            // fprintf(file,"%f, ", _rtLogger_save[i].coretemperature[j]); 	// 11 : K
            // fprintf(file,"%f, ", _rtLogger_save[i].sensortemperature[j]); 	// 12 : L
            // fprintf(file,"%i, ", _rtLogger_save[i].TargetTor[j]); 			// 13 : M
            // fprintf(file,"%i, ", _rtLogger_save[i].ActualVel[j]); 			// 14 : N
            fprintf(file,"\n");
            ++ready;
            percent_ready = (double)ready / CTRL_BUFF_SIZE * 100;
        }
        fclose(file);
        

    }
    delete[] _rtLogger_save;
    

    
}



