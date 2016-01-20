#include <wiringPiI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
 * which provides a common 'type' for sensor data and some helper functions.
 *       
 * To use this driver you will also need to download the Adafruit_Sensor
 * library and include it in your libraries folder.
 * You should also assign a unique ID to this sensor for use with
 * the Adafruit Sensor API so that you can identify this particular
 * sensor in any data logs, etc.  To assign a unique ID, simply
 * provide an appropriate value in the constructor below (12345
 * is used by default in this example).
 *                              
 * Connections
 * ===========
 * Connect SCL to analog 5
 * Connect SDA to analog 4
 * Connect VDD to 3.3V DC
 * Connect GROUND to common ground
 *                                                     
 * History
 * =======
 * 2013/JUN/17  - Updated altitude calculations (KTOWN)
 * 2013/FEB/13  - First version (KTOWN)
 * */
   

/**************************************************************************/
/*
 * Displays some basic information on this sensor from the unified
 * sensor API sensor_t type (see Adafruit_Sensor for more information)
 *         */
/**************************************************************************/
void displaySensorDetails(struct bmp_t *bmp)
{
    sensor_t sensor;
    bmp_getSensor(bmp, &sensor);
    printf("------------------------------------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %i\n", sensor.version);
    printf("Unique ID:    %i\n", sensor.sensor_id);
    printf("Max Value:    %f hPa\n", sensor.max_value);
    printf("Min Value:    %f hPa\n", sensor.max_value);
    printf("Resolution:   %f hPa\n", sensor.resolution);  
    printf("------------------------------------\n\n\n");
    sleep(2);
}

int main(void) 
{
    struct bmp_t *bmp;
    printf("Pressure Sensor Test\n");
          
    /* Initialise the sensor */
    if(!bmp_create(&bmp,123)) {
        printf("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!\n");
        return -1;
    }
            
    /* Display some basic information on this sensor */
    displaySensorDetails(bmp);
    
    for (;;) {
        /* Get a new sensor event */ 
        sensors_event_t event;
        bmp_getEvent(bmp, &event);
         
        /* Display the results (barometric pressure is measure in hPa) */
        if (event.pressure){
            /* Display atmospheric pressue in hPa */
            printf("Pressure:  %f hPa\n", event.pressure);
                                    
            /* Calculating altitude with reasonable accuracy requires pressure    *
             * sea level pressure for your position at the moment the data is     *
             * converted, as well as the ambient temperature in degress           *
             * celcius.  If you don't have these values, a 'generic' value of     *
             * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
             * in sensors.h), but this isn't ideal and will give variable         *
             * results from one day to the next.                                  *
             *
             * You can usually find the current SLP value by looking at weather   *
             * websites or from environmental information centers near any major  *
             * airport.                                                           *
             *
             * For example, for Paris, France you can check the current mean      *
             * pressure and sea level at: http://bit.ly/16Au8ol                   */
                                                 
            /* First we get the current temperature from the BMP085 */
            float temperature;
            bmp_getTemperature(bmp, &temperature);
            printf("Temperature: %f C\n",temperature);
            
            /* Then convert the atmospheric pressure, and SLP to altitude         */
            /* Update this next line with the current SLP for better results      */
            float seaLevelPressure = 1024.7f;
            printf("Altitude: %f m\n", bmp_pressureToAltitude(seaLevelPressure,event.pressure));
        }
        else {
            printf("Sensor error\n");
        }
        sleep(1);
    }
}
