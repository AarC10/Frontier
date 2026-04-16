// SPDX-License-Identifier: Apache-2.0

#ifndef CORE_SIM_SIM_SENSOR_STATE_H_
#define CORE_SIM_SIM_SENSOR_STATE_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SimSensorState {
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;
    struct sensor_value gyro_x;
    struct sensor_value gyro_y;
    struct sensor_value gyro_z;
    struct sensor_value pressure;
    struct sensor_value temperature;
};

extern struct SimSensorState g_sim_sensor_state;
extern struct k_mutex g_sim_sensor_mutex;

#ifdef __cplusplus
}
#endif

#endif
