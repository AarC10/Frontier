// SPDX-License-Identifier: Apache-2.0

#ifndef CORE_SIM_SIM_PIPE_PROTOCOL_H_
#define CORE_SIM_SIM_PIPE_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_FRAME_MAGIC   0x5752
#define SIM_FRAME_VERSION 1

struct __attribute__((packed)) SimFrame {
    uint16_t magic;
    uint8_t version;
    uint8_t reserved;
    uint32_t time_ms;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    float pressure_pa;
    float temp_c;
};

#ifdef __cplusplus
}
#endif

#endif
