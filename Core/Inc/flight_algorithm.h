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
uint8_t flight_algorithm_get_durum_verisi(void);

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

/**
 * @brief Restore flight algorithm state from backup data
 * @param phase Flight phase to restore
 * @param status_bits Status bits to restore
 * @param durum_verisi Durum verisi to restore
 * @param flight_start_time Flight start time to restore
 */
void flight_algorithm_restore_state(FlightPhase_t phase, 
                                  uint16_t status_bits, 
                                  uint8_t durum_verisi, 
                                  uint32_t flight_start_time);

/**
 * @brief Get all flight algorithm state for backup
 */
uint8_t flight_algorithm_get_is_rising(void);
uint8_t flight_algorithm_get_is_stabilized(void);
uint8_t flight_algorithm_get_is_armed(void);
uint8_t flight_algorithm_get_drogue_deployed(void);
uint8_t flight_algorithm_get_main_deployed(void);
uint8_t flight_algorithm_get_deployed_angle(void);
uint8_t flight_algorithm_get_deployed_velocity(void);
int flight_algorithm_get_apogee_counter(void);
int flight_algorithm_get_burnout_counter(void);
float flight_algorithm_get_prev_velocity(void);
uint8_t flight_algorithm_get_altitude_decrease_count(void);
float flight_algorithm_get_prev_altitude(void);
uint8_t flight_algorithm_get_drogue_pulse_active(void);
uint32_t flight_algorithm_get_drogue_pulse_start_time(void);
uint8_t flight_algorithm_get_main_pulse_active(void);
uint32_t flight_algorithm_get_main_pulse_start_time(void);

/**
 * @brief Restore complete flight algorithm state from backup data
 */
void flight_algorithm_restore_complete_state(
    FlightPhase_t phase,
    uint16_t status_bits,
    uint8_t durum_verisi,
    uint32_t flight_start_time,
    uint8_t is_rising,
    uint8_t is_stabilized,
    uint8_t is_armed,
    uint8_t drogue_deployed,
    uint8_t main_deployed,
    uint8_t deployed_angle,
    uint8_t deployed_velocity,
    int apogee_counter,
    int burnout_counter,
    float prev_velocity,
    uint8_t altitude_decrease_count,
    float prev_altitude,
    uint8_t drogue_pulse_active,
    uint32_t drogue_pulse_start_time,
    uint8_t main_pulse_active,
    uint32_t main_pulse_start_time
);

void deploy_drogue_parachute(void);
void deploy_main_parachute(void);
void deploy_parachute_update(void);
#endif /* INC_FLIGHT_ALGORITHM_H_ */
