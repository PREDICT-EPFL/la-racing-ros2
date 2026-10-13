#pragma once

#include "esp_system.h"

typedef struct {
    int8_t status[2];
    uint32_t delay[2];
    int16_t acc[3]; // status, delay, x,y,z
    int16_t gyro[3]; // status, delay, x,y,z
} __attribute__ ((packed)) raw_imu_t;

void configure_bno08x();
esp_err_t bno08x_read_packet();
esp_err_t handle_report_bno08x(raw_imu_t* imu);
