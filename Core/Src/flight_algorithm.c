/**
 * @file flight_algorithm.c
 * @brief Core flight algorithm for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#include "flight_algorithm.h"
#include "queternion.h"
#include "bme280.h"
#include "main.h"
#include <math.h>


// Private defines
#define ALT_DECREASE_THRESHOLD   3     // consecutive altitude decreases

// Private variables
static FlightPhase_t current_phase = PHASE_IDLE;

// Flight parameters (configurable)
static float launch_accel_threshold = 40.0f;  // m/s²
static float min_arming_altitude = 2000.0f;   // meters
static float main_chute_altitude = 550.0f;    // meters
static float max_angle_threshold = 75.0f;     // degrees
static float rising_vel_treshold = 30.0f;

// Flight state tracking
static uint8_t is_rising = 0;
static uint8_t is_stabilized = 1;
static uint8_t is_armed = 0;
static uint8_t deployed_angle = 1;
static uint8_t deployed_velocity = 1;
static uint8_t drogue_deployed = 0;
static uint8_t main_deployed = 0;
int apogee_counter = 0;
int burnout_counter = 1;
int launch_accel_counter = 0;
int launch_vel_counter = 0;
int altitude_arming_counter = 0;
int angle_threshold_counter = 0;
int main_altitude_counter = 0;
float prev_velocity = 0;

static uint32_t flight_start_time = 0;
static uint8_t altitude_decrease_count = 0;
static float prev_altitude = 0.0f;

// Status tracking
static uint16_t status_bits = 0;
static uint8_t durum_verisi = 1;

// Private function prototypes
static float calculate_total_acceleration(bmi088_struct_t* bmi);
void deploy_drogue_parachute(void);
void deploy_main_parachute(void);
// Drogue parachute deployment state
static uint8_t drogue_pulse_active = 0;
static uint32_t drogue_pulse_start_time = 0;

// Main parachute deployment state
static uint8_t main_pulse_active = 0;
static uint32_t main_pulse_start_time = 0;


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
    launch_accel_counter = 0;
    launch_vel_counter = 0;
    altitude_arming_counter = 0;
    angle_threshold_counter = 0;
    main_altitude_counter = 0;
    prev_velocity = 0;
    durum_verisi = 1;
    drogue_pulse_active = 0;
    drogue_pulse_start_time = 0;
    main_pulse_active = 0;
    main_pulse_start_time = 0;
    burnout_counter = 1;
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
            // Detect launch using acceleration threshold with counter
            if (total_acceleration > launch_accel_threshold) {
                launch_accel_counter++;
                if (launch_accel_counter >= 4) {
                    current_phase = PHASE_BOOST;
                    is_rising = 1;
                    flight_start_time = HAL_GetTick();
                    status_bits |= 0x0001; // Set Bit 0: Rocket launch detected
                    durum_verisi = 2;
                }
            } else {
                launch_accel_counter = 0; // Reset counter if threshold not exceeded
            }
            
            if(sensor_fusion->velocity > rising_vel_treshold){
                launch_vel_counter++;
                if (launch_vel_counter >= 4) {
                    current_phase = PHASE_BOOST;
                    is_rising = 1;
                    flight_start_time = HAL_GetTick();
                    status_bits |= 0x0001; // Set Bit 0: Rocket launch detected
                    durum_verisi = 2;
                }
            } else {
                launch_vel_counter = 0; // Reset counter if threshold not exceeded
            }
            break;

        case PHASE_BOOST:
            // After boost phase (typically 7-9 seconds)
            if (HAL_GetTick() - flight_start_time > 7000) {
                current_phase = PHASE_COAST;
                is_stabilized = 1;
                status_bits |= 0x0002; // Set Bit 1: Motor burn prevention period ended
                durum_verisi = 3;
            }
            if((-bmi->datas.acc_x) < 0.0){
            	burnout_counter++;
            	if(burnout_counter >= 4){
					current_phase = PHASE_COAST;
					is_stabilized = 1;
					status_bits |= 0x0002; // Set Bit 1: Motor burn prevention period ended
					durum_verisi = 3;
				}
            } else {
            	burnout_counter = 0;
            }
            break;

        case PHASE_COAST:
            // Check minimum arming altitude with counter
            if (bme->altitude > min_arming_altitude && !is_armed) {
                altitude_arming_counter++;
                if (altitude_arming_counter >= 4) {
                    is_armed = 1;
                    status_bits |= 0x0004; // Set Bit 2: Minimum altitude threshold exceeded
                    durum_verisi = 4;
                }
            } else {
                altitude_arming_counter = 0; // Reset counter if threshold not exceeded
            }

            // Check if angle exceeds threshold with counter
            if (is_armed && (fabs(bmi->datas.theta) > max_angle_threshold) && deployed_angle) {
                angle_threshold_counter++;
                if (angle_threshold_counter >= 4) {
                    drogue_deployed = 1;
                    deployed_angle = 0;
                    status_bits |= 0x0008; // Set Bit 3: Rocket body angle exceeds threshold
                    durum_verisi = 5;
                    deploy_drogue_parachute();
                }
            } else {
                angle_threshold_counter = 0; // Reset counter if threshold not exceeded
            }

            if (is_armed && sensor_fusion->velocity < 0.0f && deployed_velocity) {
                apogee_counter++;
                if (apogee_counter >= 4) {  // Confirm apogee after 5 consecutive samples
                    status_bits |= 0x0010; // Set Bit 4: Rocket altitude started decreasing
                    status_bits |= 0x0020; // Set Bit 5: Drag parachute deployment command generated
                    drogue_deployed = 1;
                    deployed_velocity = 0;
                    durum_verisi = 6;
                    deploy_drogue_parachute();
                }
            } else {
                apogee_counter = 0;
            }

            // Deploy main parachute at designated altitude with counter
            if (drogue_deployed && bme->altitude < main_chute_altitude) {
                main_altitude_counter++;
                if (main_altitude_counter >= 4) {
                    current_phase = PHASE_MAIN_DESCENT;
                    status_bits |= 0x0040; // Set Bit 6: Rocket altitude below specified altitude
                    status_bits |= 0x0080; // Set Bit 7: Main parachute deployment command generated
                    main_deployed = 1;
                    drogue_deployed = 0;
                    durum_verisi = 7;
                    deploy_main_parachute();
                }
            } else {
                main_altitude_counter = 0; // Reset counter if threshold not exceeded
            }
            break;

        case PHASE_MAIN_DESCENT:
            // Monitor descent under main parachute
            break;

        case PHASE_LANDED:
            // No additional status bits to set after landing
            break;
    }
    deploy_parachute_update();
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

uint8_t flight_algorithm_get_durum_verisi(void)
{
    return durum_verisi;
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

uint32_t flight_algorithm_get_start_time(void)
{
    return flight_start_time;
}

void deploy_drogue_parachute(void)
{
    if (!drogue_pulse_active) {
        HAL_GPIO_WritePin(KURTARMA1_GPIO_Port, KURTARMA1_Pin, GPIO_PIN_SET);
        drogue_pulse_start_time = HAL_GetTick();
        drogue_pulse_active = 1;
    }
}

void deploy_parachute_update(void)
{
    if (drogue_pulse_active && (HAL_GetTick() - drogue_pulse_start_time >= 1000)) {
        HAL_GPIO_WritePin(KURTARMA1_GPIO_Port, KURTARMA1_Pin, GPIO_PIN_RESET);
        drogue_pulse_active = 0;
    }
    if (main_pulse_active && (HAL_GetTick() - main_pulse_start_time >= 1000)) {
        HAL_GPIO_WritePin(KURTARMA2_GPIO_Port, KURTARMA2_Pin, GPIO_PIN_RESET);
        main_pulse_active = 0;
    }
}

void deploy_main_parachute(void)
{
    if (!main_pulse_active) {
        HAL_GPIO_WritePin(KURTARMA2_GPIO_Port, KURTARMA2_Pin, GPIO_PIN_SET);
        main_pulse_start_time = HAL_GetTick();
        main_pulse_active = 1;
    }
}

/**
 * @brief Restore flight algorithm state from backup data
 * @param phase Flight phase to restore
 * @param status_bits Status bits to restore
 * @param durum_verisi Durum verisi to restore
 * @param flight_start_time Flight start time to restore
 */
void flight_algorithm_restore_state(FlightPhase_t phase, 
                                  uint16_t status_bits_restore, 
                                  uint8_t durum_verisi_restore, 
                                  uint32_t flight_start_time_restore)
{
    current_phase = phase;
    status_bits = status_bits_restore;
    durum_verisi = durum_verisi_restore;
    flight_start_time = flight_start_time_restore;
    
    // Set appropriate flags based on status bits
    if (status_bits & BIT_LAUNCH_DETECTED) {
        is_rising = 1;
    }
    
    if (status_bits & BIT_MIN_ALTITUDE_PASSED) {
        is_armed = 1;
    }
    
    if (status_bits & (BIT_HIGH_ANGLE_OR_ACCEL | BIT_DESCENT_STARTED)) {
        drogue_deployed = 1;
        deployed_angle = 0;
        deployed_velocity = 0;
    }
    
    if (status_bits & BIT_MAIN_DEPLOYED) {
        main_deployed = 1;
        drogue_deployed = 0;
    }
}

// Getter functions for all flight algorithm state variables
uint8_t flight_algorithm_get_is_rising(void) {
    return is_rising;
}

uint8_t flight_algorithm_get_is_stabilized(void) {
    return is_stabilized;
}

uint8_t flight_algorithm_get_is_armed(void) {
    return is_armed;
}

uint8_t flight_algorithm_get_drogue_deployed(void) {
    return drogue_deployed;
}

uint8_t flight_algorithm_get_main_deployed(void) {
    return main_deployed;
}

uint8_t flight_algorithm_get_deployed_angle(void) {
    return deployed_angle;
}

uint8_t flight_algorithm_get_deployed_velocity(void) {
    return deployed_velocity;
}

int flight_algorithm_get_apogee_counter(void) {
    return apogee_counter;
}

int flight_algorithm_get_burnout_counter(void) {
    return burnout_counter;
}

float flight_algorithm_get_prev_velocity(void) {
    return prev_velocity;
}

uint8_t flight_algorithm_get_altitude_decrease_count(void) {
    return altitude_decrease_count;
}

float flight_algorithm_get_prev_altitude(void) {
    return prev_altitude;
}

uint8_t flight_algorithm_get_drogue_pulse_active(void) {
    return drogue_pulse_active;
}

uint32_t flight_algorithm_get_drogue_pulse_start_time(void) {
    return drogue_pulse_start_time;
}

uint8_t flight_algorithm_get_main_pulse_active(void) {
    return main_pulse_active;
}

uint32_t flight_algorithm_get_main_pulse_start_time(void) {
    return main_pulse_start_time;
}

/**
 * @brief Restore complete flight algorithm state from backup data
 */
void flight_algorithm_restore_complete_state(
    FlightPhase_t phase,
    uint16_t status_bits_restore,
    uint8_t durum_verisi_restore,
    uint32_t flight_start_time_restore,
    uint8_t is_rising_restore,
    uint8_t is_stabilized_restore,
    uint8_t is_armed_restore,
    uint8_t drogue_deployed_restore,
    uint8_t main_deployed_restore,
    uint8_t deployed_angle_restore,
    uint8_t deployed_velocity_restore,
    int apogee_counter_restore,
    int burnout_counter_restore,
    float prev_velocity_restore,
    uint8_t altitude_decrease_count_restore,
    float prev_altitude_restore,
    uint8_t drogue_pulse_active_restore,
    uint32_t drogue_pulse_start_time_restore,
    uint8_t main_pulse_active_restore,
    uint32_t main_pulse_start_time_restore
) {
    current_phase = phase;
    status_bits = status_bits_restore;
    durum_verisi = durum_verisi_restore;
    flight_start_time = flight_start_time_restore;
    is_rising = is_rising_restore;
    is_stabilized = is_stabilized_restore;
    is_armed = is_armed_restore;
    drogue_deployed = drogue_deployed_restore;
    main_deployed = main_deployed_restore;
    deployed_angle = deployed_angle_restore;
    deployed_velocity = deployed_velocity_restore;
    apogee_counter = apogee_counter_restore;
    burnout_counter = burnout_counter_restore;
    prev_velocity = prev_velocity_restore;
    altitude_decrease_count = altitude_decrease_count_restore;
    prev_altitude = prev_altitude_restore;
    drogue_pulse_active = drogue_pulse_active_restore;
    drogue_pulse_start_time = drogue_pulse_start_time_restore;
    main_pulse_active = main_pulse_active_restore;
    main_pulse_start_time = main_pulse_start_time_restore;
}
