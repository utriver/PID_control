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
								double const * const qdes,
								double const * const tau)
{
	// Update circular buffer
    if (!_isLogSaving)
    {
        _loggingBuff[_buffIdx].time = time;
        for (int i = 0; i < JOINT_DOF; i++)
        {
            _loggingBuff[_buffIdx].q[i] = q[i];
            _loggingBuff[_buffIdx].qdes[i] = qdes[i];
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

    const int SKIP_COUNT = 290000; // 건너뛸 데이터 개수

    while (_isRunning)
    {
        waitForEvent();

        printf("Start to Write!\n");
        _isLogSaving = true;

        FILE * logFile = NULL;
        logFile = fopen(fileName.c_str(), "a");

        if (logFile != NULL)
        {
            int totalData = _dataCount;
            int startIdx = (_buffIdx - _dataCount + BUFF_SIZE) % BUFF_SIZE;

            // 총 데이터 수가 SKIP_COUNT보다 큰지 확인
            if (totalData > SKIP_COUNT)
            {
                int dataToWrite = totalData - SKIP_COUNT;
                int skipIdx = (startIdx + SKIP_COUNT) % BUFF_SIZE;

<<<<<<< Updated upstream
                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].q[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].qdes[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
					fprintf(logFile, "%f,  ", _loggingBuff[k].tau[i]);
=======
                for (int i = 0; i < dataToWrite; i++)
                {
                    int idx = (skipIdx + i) % BUFF_SIZE;
>>>>>>> Stashed changes

                    fprintf(logFile, "%f, ", _loggingBuff[idx].time);

                    for (unsigned int j = 0; j < JOINT_DOF; j++)
                        fprintf(logFile, "%f, ", _loggingBuff[idx].q[j]);

                    for (unsigned int j = 0; j < JOINT_DOF; j++)
                        fprintf(logFile, "%f, ", _loggingBuff[idx].qdot[j]);

                    for (unsigned int j = 0; j < JOINT_DOF; j++)
                        fprintf(logFile, "%f, ", _loggingBuff[idx].tau[j]);

                    fprintf(logFile, "\n");
                }
            }
            else
            {
                printf("저장된 데이터가 290,000개보다 적습니다.\n");
            }

<<<<<<< Updated upstream
            for (int k = 0; k < _buffIdx; k++)
            {
                fprintf(logFile, "%f, ", _loggingBuff[k].time);

                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].q[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
                    fprintf(logFile, "%f,  ", _loggingBuff[k].qdes[i]);
                for (unsigned int i = 0; i < JOINT_DOF; i++)
					fprintf(logFile, "%f,  ", _loggingBuff[k].tau[i]);

                fprintf(logFile, "\n");
            }
=======
>>>>>>> Stashed changes
            fprintf(logFile, "\n\n\n");

            fclose(logFile);
            logFile = NULL;
        }

        // 로그 저장 후 _dataCount 초기화
        _dataCount = 0;
        _isLogSaving = false;
    }
}
