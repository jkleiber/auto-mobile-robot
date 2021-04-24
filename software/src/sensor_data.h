#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

typedef struct sensor_data_t {
    double imu_roll;
    double imu_pitch;
    double imu_yaw;
    double imu_linear_accel;
    double linear_velocity;
    double angular_velocity;
} SensorData;

#endif