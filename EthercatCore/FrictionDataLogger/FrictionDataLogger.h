/*
 * FrictionDataLogger.h
 *
 *  Created on: Apr 15, 2022
 *      Author: meser
 */

#ifndef FRICTIONDATALOGGER_FRICTIONDATALOGGER_H_
#define FRICTIONDATALOGGER_FRICTIONDATALOGGER_H_

#include "../COREsys.h"
#include "AgingDataLogger.h"

class FrictionDataLogger : public AgingDataLogger
{
private:
    enum{
        MAX_NUM_CORE = NUM_AXIS, DATA_LENGTH = 150,
    };
public:
    char *core_id;
public:
    FrictionDataLogger ();
    ~FrictionDataLogger ();

public:
    void write_rt_buffer(int filenum, double &percent_ready);

};




#endif /* FRICTIONDATALOGGER_FRICTIONDATALOGGER_H_ */
