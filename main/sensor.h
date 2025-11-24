#ifndef _SENSOR_H
#define _SENSOR_H

#define emalloc(size) malloc(size)
// as5600
enum SensorType {
    AS5600
};

struct Sensor {
    enum SensorType type;
    void *sensor;

    // function
    void (*update)(struct Sensor *sensor);
    float (*read_angle_without_track)(struct Sensor *sensor);
    float (*read_angle)(struct Sensor *sensor);
    float (*read_velocity)(struct Sensor *sensor);
};

#endif