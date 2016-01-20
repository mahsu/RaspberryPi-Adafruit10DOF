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
    
    struct accel_t *accel;
    sensor_t sensor;
    printf("Accelerometer Test\n");

    /* Initialise the sensor */
    if(!accel_create(&accel, 54321)) {
        printf("No LSM303 detected!");
        return -1;
    }
            
    // Display sensor information
    accel_getSensor(accel,&sensor);
    printf("------------------------------------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %i\n", sensor.version);
    printf("Unique ID:    %i\n", sensor.sensor_id);
    printf("Max Value:    %f m/s^2\n", sensor.max_value);
    printf("Min Value:    %f m/s^2\n", sensor.max_value);
    printf("Resolution:   %f m/s^2\n", sensor.resolution);  
    printf("------------------------------------\n\n\n");
    sleep(2);

    for (;;) {
        /* Get a new sensor event */ 
        sensors_event_t event; 
        accel_getEvent(accel, &event); 
        printf("X: % 010.6f   ", event.acceleration.x);
        printf("Y: % 010.6f   ", event.acceleration.y);
        printf("Z: % 010.6f   ", event.acceleration.z);
        printf(" m/s^2 \n");
        usleep(100000);
    }

    return 0;
}
