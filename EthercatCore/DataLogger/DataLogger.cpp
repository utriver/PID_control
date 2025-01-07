#include "DataLogger.h"

DataLogger::DataLogger()
: _isRunning(false)
, _isLogSaving(false)
, _logEvent(false)
, _buffIdx(0)
{
}

DataLogger::~DataLogger()
{
    deactivate();
}

bool DataLogger::activate()
{
    _isRunning = true;
    _thread.start(*this);
    return true;
}

bool DataLogger::deactivate()
{
    Poco::Thread::sleep(1000);
    _isRunning = false;
    _logEvent = true;

    if (_thread.isRunning())
    {
        _thread.yield();
        _thread.join();
    }
    return true;
}


void DataLogger::updateLoggedData(const double time,
								double const * const q,
								double const * const qdot,
								double const * const tau)
{
	// Update circular buffer
    if (!_isLogSaving)
    {
        _loggingBuff[_buffIdx].time = time;
        for (int i = 0; i < JOINT_DOF; i++)
        {
            _loggingBuff[_buffIdx].q[i] = q[i];
            _loggingBuff[_buffIdx].qdot[i] = qdot[i];
            _loggingBuff[_buffIdx].tau[i] = tau[i];
        }

        _buffIdx++;
        if (_buffIdx >= BUFF_SIZE)
            _buffIdx = 0;
    }
}

void DataLogger::triggerSaving()
{
    _logEvent = true;
}

void DataLogger::waitForEvent()
{
    while (!_logEvent)
    {
        Poco::Thread::sleep(10);
    }
    _logEvent = false;
}

void DataLogger::run()
{
    std::string fileName = SAMPLE_LOG_FILE;

    while (_isRunning)
    {
        waitForEvent();

        printf("Start to Write!\n");
        _isLogSaving = true;

        FILE * logFile = NULL;
        logFile = fopen(fileName.c_str(), "a");

        if (logFile != NULL)
        {
            for (int k = _buffIdx; k < BUFF_SIZE; k++)
            {
                fprintf(logFile, "%f, ", _loggingBuff[k].time);

                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].q[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].qdot[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
					fprintf(logFile, "%f,  ", _loggingBuff[k].tau[i]);

                fprintf(logFile, "\n");
            }

            for (int k = 0; k < _buffIdx; k++)
            {
                fprintf(logFile, "%f, ", _loggingBuff[k].time);

                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].q[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].qdot[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
					fprintf(logFile, "%f,  ", _loggingBuff[k].tau[i]);

                fprintf(logFile, "\n");
            }
            fprintf(logFile, "\n\n\n");

            fclose(logFile);
            logFile = NULL;
        }

        _isLogSaving = false;
    }
}
