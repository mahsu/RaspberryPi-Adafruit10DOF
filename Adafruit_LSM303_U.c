/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  Ported by Matthew Hsu to Raspberry Pi compatible C for Cornell Rocketry Team.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <limits.h>
#include <wiringPiI2C.h>
#include "Adafruit_LSM303_U.h"

// enabling this #define will enable the debug print blocks
//#define LSM303_DEBUG


static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void _accel_read(struct accel_t *accel) {

	int fd = accel->fd;
  
    // Read the accelerometer

    uint8_t xlo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_X_L_A);
    uint8_t xhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_X_H_A);
    uint8_t ylo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_Y_L_A);
    uint8_t yhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_Y_H_A);
    uint8_t zlo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_Z_L_A);
    uint8_t zhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_ACCEL_OUT_Z_H_A);
  
    // Shift values to create properly formed integer (low byte first)
    accel->accelData.x = (int16_t)(xlo | (xhi << 8)) >> 4;
    accel->accelData.y = (int16_t)(ylo | (yhi << 8)) >> 4;
    accel->accelData.z = (int16_t)(zlo | (zhi << 8)) >> 4;
}

 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool accel_create(struct accel_t **ret_accel, int32_t sensorID)
{
  struct accel_t *accel = malloc (sizeof (struct accel_t));

  // Enable I2C
  int fd = wiringPiI2CSetup(LSM303_ADDRESS_ACCEL);
  if (fd == -1) {
	  return false;
  }

  //save the i2c handle for later
  accel->fd = fd;
 
  // default to return data in m/s^2 instead of G's
  accel->useEarthGravity = true;

  // Enable the accelerometer (100Hz)
  wiringPiI2CWriteReg8(fd, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
  
  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = wiringPiI2CReadReg8(fd, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
  if (reg1_a != 0x57)
  {
    return false;
  }  
  
  *ret_accel = accel;
  return true;
}


/**************************************************************************/
/*!
    @brief Whether to return sensor data in G's (false) or m/s^2 (true) 
*/
/**************************************************************************/
void accel_useEarthGravity( struct accel_t *accel, bool useEarthGravity)
{
    accel->useEarthGravity = useEarthGravity;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool accel_getEvent(struct accel_t *accel, sensors_event_t* event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));
  
  /* Read new data */
  _accel_read(accel);

  lsm303AccelData data = accel->accelData;

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = accel->sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;//TODO implement millis //millis();
  event->acceleration.x = data.x * _lsm303Accel_MG_LSB;
  event->acceleration.y = data.y * _lsm303Accel_MG_LSB;
  event->acceleration.z = data.z * _lsm303Accel_MG_LSB;

  if (accel->useEarthGravity) {
    event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
    event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
    event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
  }

  return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void accel_getSensor(struct accel_t *accel, sensor_t* sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = accel->sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void _mag_read(struct mag_t *mag)
{
    int fd = mag->fd;
    
    // Read the magnetometer
    uint8_t xlo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_X_L_M);
    uint8_t xhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_X_H_M);
    uint8_t ylo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_Y_L_M);
    uint8_t yhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_Y_H_M);
    uint8_t zlo = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_Z_L_M);
    uint8_t zhi = (uint8_t)wiringPiI2CReadReg8(fd,LSM303_REGISTER_MAG_OUT_Z_H_M);
  
  // Shift values to create properly formed integer (low byte first)
  mag->magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
  mag->magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
  mag->magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));
  
  // ToDo: Calculate orientation
  // _magData.orientation = 0.0;
}

 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool mag_create( struct mag_t **ret_mag, int32_t sensorID)
{
  struct mag_t *mag = malloc (sizeof (struct mag_t));

  // Enable I2C
  //Wire.begin();
  int fd = wiringPiI2CSetup(LSM303_ADDRESS_MAG);
  if (fd == -1) {
	  return false;
  }
 
  //save the i2c handle for later
  mag->fd = fd;

  // Enable the magnetometer
  wiringPiI2CWriteReg8(fd, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
  // the default value (0b00010000/0x10)
  uint8_t reg1_a = wiringPiI2CReadReg8(fd, LSM303_REGISTER_MAG_CRA_REG_M);
  if (reg1_a != 0x10)
  {
    return false;
  }

  // Set the gain to a known level
  mag_setGain(mag, LSM303_MAGGAIN_1_3);

  *ret_mag = mag;
  return true;
}

/**************************************************************************/
/*! 
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void mag_enableAutoRange(struct mag_t *mag, bool enabled)
{
  mag->autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void mag_setGain(struct mag_t *mag, lsm303MagGain gain)
{
  wiringPiI2CWriteReg8(mag->fd, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);
  
  mag->magGain = gain;
 
  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
      _lsm303Mag_Gauss_LSB_Z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      _lsm303Mag_Gauss_LSB_XY = 855;
      _lsm303Mag_Gauss_LSB_Z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      _lsm303Mag_Gauss_LSB_XY = 670;
      _lsm303Mag_Gauss_LSB_Z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      _lsm303Mag_Gauss_LSB_XY = 450;
      _lsm303Mag_Gauss_LSB_Z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      _lsm303Mag_Gauss_LSB_XY = 400;
      _lsm303Mag_Gauss_LSB_Z  = 355;
      break;
    case LSM303_MAGGAIN_5_6:
      _lsm303Mag_Gauss_LSB_XY = 330;
      _lsm303Mag_Gauss_LSB_Z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      _lsm303Mag_Gauss_LSB_XY = 230;
      _lsm303Mag_Gauss_LSB_Z  = 205;
      break;
  } 
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void mag_setRate(struct mag_t *mag, lsm303MagRate rate)
{
	byte reg_m = ((byte)rate & 0x07) << 2;
    wiringPiI2CWriteReg8(mag->fd, LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}


/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool mag_getEvent(struct mag_t *mag, sensors_event_t* event) {
  bool readingValid = false;
  
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));
  
  while(!readingValid)
  {

    uint8_t reg_mg = wiringPiI2CReadReg8(mag->fd, LSM303_REGISTER_MAG_SR_REG_Mg);
    if (!(reg_mg & 0x1)) {
			return false;
    }
  
    /* Read new data */
    _mag_read(mag);
    
    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!mag->autoRangeEnabled)
    {
      readingValid = true;
    }
    else
    {
#ifdef LSM303_DEBUG
      printf("%f ", mag->magData.x);
      printf("%f ", mag->magData.y);
      printf("%f \n", mag->magData.z);
#endif	  
      /* Check if the sensor is saturating or not */
      lsm303MagData data = mag->magData;
      if ( (data.x >= 2040) | (data.x <= -2040) | 
           (data.y >= 2040) | (data.y <= -2040) | 
           (data.z >= 2040) | (data.z <= -2040) )
      {
        /* Saturating .... increase the range if we can */
        switch(mag->magGain)
        {
          case LSM303_MAGGAIN_5_6:
            mag_setGain(mag, LSM303_MAGGAIN_8_1);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 8.1\n");
#endif
            break;
          case LSM303_MAGGAIN_4_7:
            mag_setGain(mag, LSM303_MAGGAIN_5_6);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 5.6\n");
#endif
            break;
          case LSM303_MAGGAIN_4_0:
            mag_setGain(mag, LSM303_MAGGAIN_4_7);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 4.7\n");
#endif			
            break;
          case LSM303_MAGGAIN_2_5:
            mag_setGain(mag, LSM303_MAGGAIN_4_0);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 4.0\n");
#endif			
            break;
          case LSM303_MAGGAIN_1_9:
            mag_setGain(mag, LSM303_MAGGAIN_2_5);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 2.5\n");
#endif			
            break;
          case LSM303_MAGGAIN_1_3:
            mag_setGain(mag, LSM303_MAGGAIN_1_9);
            readingValid = false;
#ifdef LSM303_DEBUG
            printf("Changing range to +/- 1.9\n");
#endif			
            break;
          default:
            readingValid = true;
            break;  
        }
      }
      else
      {
        /* All values are withing range */
        readingValid = true;
      }
    }
  }
  
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = mag->sensorID;
  event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = 0; //TODO implement millis();
  event->magnetic.x = mag->magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = mag->magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = mag->magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
		
	return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void mag_getSensor(struct mag_t *mag, sensor_t *sensor){
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = mag->sensorID;
  sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}
