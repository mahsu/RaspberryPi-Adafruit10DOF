#include <wiringPiI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

int main() 
{
    
    struct mag_t *mag;
    sensor_t sensor;
    printf("Magnetometer Test\n");

    /* Initialise the sensor */
    if(!mag_create(&mag, 54321)) {
        printf("No LSM303 detected!");
        return -1;
    }

    mag_enableAutoRange(mag, true);
            
    // Display sensor information
    mag_getSensor(mag,&sensor);
    printf("------------------------------------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %i\n", sensor.version);
    printf("Unique ID:    %i\n", sensor.sensor_id);
    printf("Max Value:    %f uT\n", sensor.max_value);
    printf("Min Value:    %f uT\n", sensor.max_value);
    printf("Resolution:   %f uT\n", sensor.resolution);  
    printf("------------------------------------\n\n\n");
    sleep(2);

    for (;;) {
        /* Get a new sensor event */ 
        sensors_event_t event; 
        mag_getEvent(mag, &event); 
        printf("X: % 010.6f   ", event.magnetic.x);
        printf("Y: % 010.6f   ", event.magnetic.y);
        printf("Z: % 010.6f   ", event.magnetic.z);
        printf(" uT \n");
        usleep(100000);
    }

    return 0;
}
