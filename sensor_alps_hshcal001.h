

#ifndef SENSOR_ALPS_HSHCAL001_H__
#define SENSOR_ALPS_HSHCAL001_H__

#include "sensor.h"
#include "hshcal001.h"

#define HSHCAL001_ADDR_DEFAULT (0x18)

int rt_hw_hshcal001_init(const char *name, struct rt_sensor_config *cfg);

#endif