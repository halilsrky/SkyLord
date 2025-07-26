/**
 * @file flight_algorithm.c
 * @brief Core flight algorithm for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#include "flight_algorithm.h"
#include "queternion.h"
#include "bme280.h"  // This will include main.h which has HAL definitions
#include <math.h>

// External dependencies
extern int is_BME_ok;

// Private defines
#define ALT_DECREASE_THRESHOLD   3     // consecutive altitude decreases

// Private variables
static FlightPhase_t current_phase = PHASE_IDLE;

// Flight parameters (configurable)
static float launch_accel_threshold = 30.0f;  // m/s²
static float min_arming_altitude = 2000.0f;   // meters
static float main_chute_altitude = 500.0f;    // meters
static float max_angle_threshold = 70.0f;     // degrees

// Flight state tracking
static uint8_t is_rising = 0;
static uint8_t is_stabilized = 1;
static uint8_t is_armed = 0;
static uint8_t deployed_angle = 1;
static uint8_t deployed_velocity = 1;
static uint8_t drogue_deployed = 0;
static uint8_t main_deployed = 0;
int apogee_counter = 0;
float prev_velocity = 0;

static uint32_t flight_start_time = 0;
static uint8_t altitude_decrease_count = 0;
static float prev_altitude = 0.0f;

// Status tracking
static uint16_t status_bits = 0;

// Private function prototypes
static float calculate_total_acceleration(bmi088_struct_t* bmi);

/**
 * @brief Initialize the flight algorithm
 */
void flight_algorithm_init(void)
{
    flight_algorithm_reset();
}

/**
 * @brief Reset flight algorithm to initial state
 */
void flight_algorithm_reset(void)
{
    current_phase = PHASE_IDLE;
    is_rising = 0;
    is_stabilized = 1;
    is_armed = 0;
    drogue_deployed = 0;
    main_deployed = 0;
    altitude_decrease_count = 0;
    status_bits = 0;
    prev_altitude = 0.0f;
    flight_start_time = 0;
    deployed_velocity = 1;
    deployed_angle = 1;
    apogee_counter = 0;
    prev_velocity = 0;
}

/**
 * @brief Update flight algorithm with sensor data
 */
void flight_algorithm_update(BME_280_t* bme, bmi088_struct_t* bmi, sensor_fusion_t* sensor_fusion)
{
    // Calculate key metrics
    float total_acceleration = calculate_total_acceleration(bmi);
    //float theta = sensor_fusion->angle; // Use sensor fusion output

    // Status bits are cumulative - once set they remain set
    // Each phase builds on the previous phase's status bits

    // State machine for flight phases
    switch (current_phase) {
        case PHASE_IDLE:
            // Detect launch using acceleration threshold
            if (total_acceleration > launch_accel_threshold) {
                current_phase = PHASE_BOOST;
                is_rising = 1;
                flight_start_time = HAL_GetTick();
                status_bits |= 0x0001; // Set Bit 0: Rocket launch detected
            }
            break;

        case PHASE_BOOST:
            // After boost phase (typically 3-5 seconds)
            if (HAL_GetTick() - flight_start_time > 7000) {
                current_phase = PHASE_COAST;
                is_stabilized = 1;
                status_bits |= 0x0002; // Set Bit 1: Motor burn prevention period ended
            }
            break;

        case PHASE_COAST:
            // Check minimum arming altitude
            if (bme->altitude > min_arming_altitude) {
                is_armed = 1;
                status_bits |= 0x0004; // Set Bit 2: Minimum altitude threshold exceeded
            }

            if (sensor_fusion->filtered_altitude > min_arming_altitude) {
                                        is_armed = 1;
                                        status_bits |= 0x0008; // Set Bit 2: Minimum altitude threshold exceeded
			}

            // Check if angle exceeds threshold
            if (is_armed && (fabs(bmi->datas.angle_y) > max_angle_threshold) && deployed_angle) {
            	drogue_deployed = 1;
            	deployed_angle = 0;
                status_bits |= 0x0008; // Set Bit 3: Rocket body angle exceeds threshold
            }

       /*     // Detect altitude decrease (apogee)
            if (is_armed && sensor_fusion->apogeeDetect == 1 && deployed_velocity) {
				status_bits |= 0x0010; // Set Bit 4: Rocket altitude started decreasing
				status_bits |= 0x0020; // Set Bit 5: Drag parachute deployment command generated
				drogue_deployed = 1;
				deployed_velocity = 0;
				// deploy_drogue_parachute(); // Actual deployment command
            }*/

            if (is_armed && sensor_fusion->velocity < 0.0f && sensor_fusion->velocity < prev_velocity && deployed_velocity) {
				apogee_counter++;
				if (apogee_counter >= 5) {  // Confirm apogee after 5 consecutive samples
					status_bits |= 0x0010; // Set Bit 4: Rocket altitude started decreasing
					status_bits |= 0x0020; // Set Bit 5: Drag parachute deployment command generated
					drogue_deployed = 1;
					deployed_velocity = 0;
				}
			} else {
				apogee_counter = 0;
			}
			prev_velocity = sensor_fusion->velocity;

            // Deploy main parachute at designated altitude
            if (drogue_deployed && bme->altitude < main_chute_altitude) {
                current_phase = PHASE_MAIN_DESCENT;
                status_bits |= 0x0040; // Set Bit 6: Rocket altitude below specified altitude
                status_bits |= 0x0080; // Set Bit 7: Main parachute deployment command generated
                main_deployed = 1;
                drogue_deployed = 0;
                // deploy_main_parachute(); // Actual deployment command
            }
            break;

        case PHASE_MAIN_DESCENT:
            // Monitor descent under main parachute
            break;

        case PHASE_LANDED:
            // No additional status bits to set after landing
            break;
    }

    prev_altitude = bme->altitude;
}


/**
 * @brief Calculate total acceleration magnitude
 */
static float calculate_total_acceleration(bmi088_struct_t* bmi)
{
    return sqrtf((bmi->datas.acc_x * bmi->datas.acc_x) +
                 (bmi->datas.acc_y * bmi->datas.acc_y) +
                 (bmi->datas.acc_z * bmi->datas.acc_z));
}

/**
 * @brief Get the current flight phase
 */
FlightPhase_t flight_algorithm_get_phase(void)
{
    return current_phase;
}

/**
 * @brief Get the current status bits
 */
uint16_t flight_algorithm_get_status_bits(void)
{
    return status_bits;
}

/**
 * @brief Set flight parameters
 */
void flight_algorithm_set_parameters(float launch_accel_threshold_param,
                                   float min_arming_altitude_param,
                                   float main_chute_altitude_param,
                                   float max_angle_threshold_param)
{
    launch_accel_threshold = launch_accel_threshold_param;
    min_arming_altitude = min_arming_altitude_param;
    main_chute_altitude = main_chute_altitude_param;
    max_angle_threshold = max_angle_threshold_param;
}

/**
 * @brief Get flight start time in milliseconds
 * @return Flight start time from HAL_GetTick(), or 0 if flight hasn't started
 */
uint32_t flight_algorithm_get_start_time(void)
{
    return flight_start_time;
}
