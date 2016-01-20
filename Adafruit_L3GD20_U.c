/***************************************************
  This is a library for the L3GD20 GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  Ported by Matthew Hsu  to Raspberry Pi compatible C for Cornell Rocketry Team.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <limits.h>
#include <wiringPiI2C.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_L3GD20_U.h"
#include "common.h"

 
/**************************************************************************/
/*!
    @brief  Sets up a new gyro_t sensor, instantiating a connection
*/
/**************************************************************************/
bool gyro_create( struct gyro_t **ret_gyro, int32_t sensorID, gyroRange_t rng) {
    struct gyro_t *gyro = malloc (sizeof(struct gyro_t));
    gyro->sensorID = sensorID;
    gyro->autoRangeEnabled = false;
    gyro->range = rng; //Set the range to an appropriate value 

  /* Enable I2C */
  int fd = wiringPiI2CSetup(L3GD20_ADDRESS);
  if (fd == -1) {
	  return false;
  }

  //save the i2c handle for later
  gyro->fd = fd;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = wiringPiI2CReadReg8(fd, GYRO_REGISTER_WHO_AM_I);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
  {
    return false;
  }

  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Reset then switch to normal mode and enable all three channels */
  wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x00);
  wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(rng)
  {
    case GYRO_RANGE_250DPS:
      wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG4, 0x00);
      break;
    case GYRO_RANGE_500DPS:
      wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG4, 0x10);
      break;
    case GYRO_RANGE_2000DPS:
      wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */
  *ret_gyro = gyro;
  return true;

}

/**************************************************************************/
/*! 
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void gyro_enableAutoRange(struct gyro_t *gyro, bool enabled)
{
  gyro->autoRangeEnabled = enabled;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool gyro_getEvent(struct gyro_t *gyro, sensors_event_t* event)
{
  int fd = gyro->fd;
  bool readingValid = false;
  
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = gyro->sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  
  while(!readingValid)
  {
      uint8_t xlo = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_X_L);
      uint8_t xhi = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_X_H);
      uint8_t ylo = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_Y_L);
      uint8_t yhi = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_Y_H);
      uint8_t zlo = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_Z_L);
      uint8_t zhi = (uint8_t)wiringPiI2CReadReg8(fd,GYRO_REGISTER_OUT_Z_H);
    
    /* Shift values to create properly formed integer (low byte first) */
    event->gyro.x = (int16_t)(xlo | (xhi << 8));
    event->gyro.y = (int16_t)(ylo | (yhi << 8));
    event->gyro.z = (int16_t)(zlo | (zhi << 8));
    
    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!(gyro->autoRangeEnabled))
    {
      readingValid = true;
    }
    else
    {
      /* Check if the sensor is saturating or not */
      if ( (event->gyro.x >= 32760) | (event->gyro.x <= -32760) | 
           (event->gyro.y >= 32760) | (event->gyro.y <= -32760) | 
           (event->gyro.z >= 32760) | (event->gyro.z <= -32760) )
      {
        /* Saturating .... increase the range if we can */
        switch(gyro->range)
        {
          case GYRO_RANGE_500DPS:
            /* Push the range up to 2000dps */
            gyro->range = GYRO_RANGE_2000DPS;
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x00);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x0F);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG4, 0x20);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 2000DPS");
            break;
          case GYRO_RANGE_250DPS:
            /* Push the range up to 500dps */
            gyro->range = GYRO_RANGE_500DPS;
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x00);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG1, 0x0F);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG4, 0x10);
            wiringPiI2CWriteReg8(fd, GYRO_REGISTER_CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 500DPS");
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
  
  /* Compensate values depending on the resolution */
  switch(gyro->range)
  {
    case GYRO_RANGE_250DPS:
      event->gyro.x *= GYRO_SENSITIVITY_250DPS;
      event->gyro.y *= GYRO_SENSITIVITY_250DPS;
      event->gyro.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      event->gyro.x *= GYRO_SENSITIVITY_500DPS;
      event->gyro.y *= GYRO_SENSITIVITY_500DPS;
      event->gyro.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_2000DPS:
      event->gyro.x *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.y *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }
  
  /* Convert values to rad/s */
  event->gyro.x *= SENSORS_DPS_TO_RADS;
  event->gyro.y *= SENSORS_DPS_TO_RADS;
  event->gyro.z *= SENSORS_DPS_TO_RADS;
  
  return true;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void gyro_getSensor(struct gyro_t *gyro, sensor_t* sensor)
{  
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "L3GD20", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = gyro->sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = (float)gyro->range * SENSORS_DPS_TO_RADS;
  sensor->min_value   = (gyro->range * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution  = 0.0F; // TBD
}
