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
    //get sensor details
    //gyro_getSensor(&sensor);

    // Initiailiza sensor
    
    if (!gyro_create(&gyro, 20, GYRO_RANGE_250DPS)) {
        printf("No L3GD20 detected!");
        return;
    }
    gyro_enableAutoRange(gyro, true);

    //printf("Sensor %s", sensor.name);
    
    for (;;) {
        sensors_event_t event;
        gyro_getEvent(gyro, &event);
        printf(" X:%f Y:%f Z:%f \n",event.gyro.x,event.gyro.y,event.gyro.z);
        usleep(500000);
    }
}
