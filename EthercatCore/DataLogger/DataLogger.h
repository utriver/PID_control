#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <Poco/Runnable.h>
#include <Poco/Thread.h>

#include "DataConfiguration.h"

#define BUFF_SIZE       	30*4000  //5sec * 4000Hz
#define SAMPLE_LOG_FILE     "/home/user/release/SampleLoggingFile.csv"

class DataLogger : public Poco::Runnable
{
    enum
    {
        JOINT_DOF = 1
    };
private:
    bool _isRunning;
    bool _isLogSaving;
    bool _logEvent;

    LoggedData _loggingBuff[BUFF_SIZE];
    unsigned _buffIdx;
    unsigned _dataCount;

protected:
    Poco::Thread _thread;

public:
    DataLogger();
    virtual ~DataLogger();

    bool activate();
    bool deactivate();

    void updateLoggedData(const double time, double const * const q, double const * const qdot, double const * const tau);
    void triggerSaving();
    void waitForEvent();

    virtual void run();
};

#endif /* DATALOGGER_H_ */
