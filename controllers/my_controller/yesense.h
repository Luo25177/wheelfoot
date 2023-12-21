#pragma once

#include "datastruct.h"
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/gps.h>
#include "robotparam.h"

typedef struct {
	WbDeviceTag gps;
	WbDeviceTag imu;
	WbDeviceTag gyro;
	WbDeviceTag accel;

	float accelx;
	float accely;
	float accelz;

	datastruct pitch;
	datastruct roll;
	datastruct yaw;

	float x;
	float v;
}Yesense;

void yesenseUpdata(Yesense* yesense);
void yesenseInit(Yesense* yesense);

