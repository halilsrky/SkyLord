/**
 * @file flight_algorithm.h
 * @brief Core flight algorithm for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#ifndef INC_FLIGHT_ALGORITHM_H_
#define INC_FLIGHT_ALGORITHM_H_

#include "bme280.h"
#include "bmi088.h"
#include "sensor_fusion.h"
#include <stdint.h>

// Flight Status Bits
#define BIT_LAUNCH_DETECTED       0x0001  // Launch detected by acceleration threshold
#define BIT_BURNOUT_TIMEOUT       0x0002  // Burnout timeout period elapsed
#define BIT_MIN_ALTITUDE_PASSED   0x0004  // Minimum arming altitude passed
#define BIT_HIGH_ANGLE_OR_ACCEL   0x0008  // High angle detected for drogue deployment
#define BIT_DESCENT_STARTED       0x0010  // Descent detected for drogue deployment
#define BIT_BELOW_MAIN_ALTITUDE   0x0020  // Below main parachute deployment altitude
#define BIT_LANDED                0x0040  // Rocket landed (not currently used)
#define BIT_SENSOR_ERROR          0x0080  // Sensor error detected
#define BIT_DROGUE_DEPLOYED       0x0100  // Drogue parachute deployed
#define BIT_MAIN_DEPLOYED         0x0200  // Main parachute deployed

// Flight phases
typedef enum {
    PHASE_IDLE,             // Pre-launch, on pad
    PHASE_BOOST,            // Rocket engines burning
    PHASE_COAST,            // After burnout, still ascending
    PHASE_MAIN_DESCENT,     // Descent with main parachute
    PHASE_LANDED            // Landed
} FlightPhase_t;

/**
 * @brief Initialize the flight algorithm
 */
void flight_algorithm_init(void);

/**
 * @brief Reset flight algorithm to initial state
 */
void flight_algorithm_reset(void);

/**
 * @brief Update flight algorithm with sensor data
 * @param bme Pointer to BME280 data structure
 * @param bmi Pointer to BMI088 data structure
 * @param sensor_fusion Pointer to sensor fusion output
 * @return Current status bits
 */
void flight_algorithm_update(BME_280_t* bme, bmi088_struct_t* bmi, sensor_fusion_t* sensor_fusion);
uint32_t flight_algorithm_get_start_time(void);
/**
 * @brief Get the current flight phase
 * @return Current flight phase
 */
FlightPhase_t flight_algorithm_get_phase(void);

/**
 * @brief Get the current status bits
 * @return Status bits as a 16-bit value
 */
uint16_t flight_algorithm_get_status_bits(void);

/**
 * @brief Set flight parameters
 * @param launch_accel_threshold Launch detection acceleration threshold (m/s²)
 * @param min_arming_altitude Minimum altitude for arming (m)
 * @param main_chute_altitude Main parachute deployment altitude (m)
 * @param max_angle_threshold Maximum angle threshold for drogue deployment (degrees)
 */
void flight_algorithm_set_parameters(float launch_accel_threshold,
                                    float min_arming_altitude,
                                    float main_chute_altitude,
                                    float max_angle_threshold);


void deploy_drogue_parachute(void);
void deploy_main_parachute(void);
void deploy_parachute_update(void);
#endif /* INC_FLIGHT_ALGORITHM_H_ */
