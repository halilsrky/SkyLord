/**
 * @file sensor_fusion.h
 * @brief Sensor fusion interface for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include "bme280.h"
#include "bmi088.h"
#include <stdint.h>

/**
 * @brief Sensor fusion output structure
 */
typedef struct {
    float filtered_altitude;   // Kalman filtered altitude
    float velocity;
    float angle;              // Theta angle from Mahony filter
    float yaw;                // Yaw angle
    float pitch;              // Pitch angle
    float roll;               // Roll angle
    uint8_t apogeeDetect;
    uint8_t accel_failure;    // İvme sensörü arıza bayrağı
} sensor_fusion_t;

void sensor_fusion_init();
void sensor_fusion_update_kalman(BME_280_t* BME, bmi088_struct_t* BMI, sensor_fusion_t* sensor);


#endif /* INC_SENSOR_FUSION_H_ */
