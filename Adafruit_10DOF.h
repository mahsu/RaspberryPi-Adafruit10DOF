/***************************************************************************
  This is a library for the Adafruit 10DOF Breakout

  Designed specifically to work with the Adafruit 10DOF Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __ADAFRUIT_10DOF_H__
#define __ADAFRUIT_10DOF_H__

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <wiringPiI2C.h>
#include <common.h>

/** Sensor axis */
typedef enum
{
  SENSOR_AXIS_X  = (1),
  SENSOR_AXIS_Y  = (2),
  SENSOR_AXIS_Z  = (3)
} sensors_axis_t;

/* Driver for the the 10DOF breakout sensors */
bool ada10dof_accelGetOrientation  ( sensors_event_t *event, sensors_vec_t *orientation );
bool ada10dof_magTiltCompensation  ( sensors_axis_t axis, sensors_event_t *mag_event, sensors_event_t *accel_event );
bool ada10dof_magGetOrientation    ( sensors_axis_t axis, sensors_event_t *event, sensors_vec_t *mag_orientation );
bool ada10dof_fusionGetOrientation ( sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation );

#endif
