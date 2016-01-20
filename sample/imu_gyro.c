#include <wiringPiI2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
struct gyro_t *gyro;


int main() {
    struct gyro_t *gyro;
    sensor_t sensor;
    
    printf("Gyroscope Test\n");
    
    // Initiailize the sensor 
    if (!gyro_create(&gyro, 20, GYRO_RANGE_250DPS)) {
        printf("No L3GD20 detected!");
        return -1;
    }

    // Enable auto-ranging
    gyro_enableAutoRange(gyro, true);

    
    // Display sensor information
    gyro_getSensor(gyro, &sensor);
    printf("------------------------------------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %i\n", sensor.version);
    printf("Unique ID:    %i\n", sensor.sensor_id);
    printf("Max Value:    %f rad/s\n", sensor.max_value);
    printf("Min Value:    %f rad/s\n", sensor.min_value);
    printf("Resolution:   %f rad/s\n", sensor.resolution);  
    printf("------------------------------------\n\n\n");
    sleep(2);

    for (;;) {
        sensors_event_t event;
        gyro_getEvent(gyro, &event);
        printf("X: % 010.6f   ", event.gyro.x);
        printf("Y: % 010.6f   ", event.gyro.y);
        printf("Z: % 010.6f   ", event.gyro.z);
        printf(" rad/s \n");
        usleep(100000);
    }

    return 0;
}
