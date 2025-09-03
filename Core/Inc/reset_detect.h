/*
 * reset_detect.h
 *
 *  Created on: Aug 22, 2024
 *      Author: yahya
 */

#ifndef INC_RESET_DETECT_H_
#define INC_RESET_DETECT_H_
#include "main.h"
#include "stdlib.h"
#include "bme280.h"
#include "bmi088.h"
#include "flight_algorithm.h"
#include "uart_handler.h"
#include "queternion.h"

extern RTC_HandleTypeDef hrtc;

// Magic number for backup SRAM validity check
#define BACKUP_SRAM_MAGIC_NUMBER 0xABCD1234

// Flight algorithm critical data structure
typedef struct {
    FlightPhase_t current_phase;
    uint16_t status_bits;
    uint8_t durum_verisi;
    uint8_t is_armed;
    uint8_t drogue_deployed;
    uint8_t main_deployed;
    uint32_t flight_start_time;
    float base_altitude;
    float quaternion[4];                // [w, x, y, z] latest quaternion values
    SystemMode_t test_mode;             // Current test mode (NORMAL, SIT, SUT)
    
    // Additional flight algorithm state variables
    uint8_t is_rising;
    uint8_t is_stabilized;
    uint8_t deployed_angle;
    uint8_t deployed_velocity;
    int apogee_counter;
    int burnout_counter;
    float prev_velocity;
    uint8_t altitude_decrease_count;
    float prev_altitude;
    uint8_t drogue_pulse_active;
    uint32_t drogue_pulse_start_time;
    uint8_t main_pulse_active;
    uint32_t main_pulse_start_time;
} flight_critical_data_t;

// Complete backup SRAM data structure
typedef struct {
    uint32_t magic_number;              // Validity check
    BME_parameters_t bme_params;        // BME280 calibration parameters
    bmi088_offsets_t bmi_offsets;       // BMI088 offset data
    flight_critical_data_t flight_data; // Flight algorithm critical data
    float base_altitude;                // Base altitude for relative calculations
    uint32_t last_save_timestamp;       // Last save timestamp
} backup_sram_data_t;

// Function prototypes
void save_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t);
uint32_t measure_abs_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t);

// New backup SRAM functions
void backup_sram_init(void);
uint8_t is_reset_occurred(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t);
void save_static_calibration_data(BME_280_t* bme, bmi088_struct_t* bmi);
uint8_t restore_critical_data_from_backup_sram(BME_280_t* bme, bmi088_struct_t* bmi);
void save_flight_data_to_backup_sram(void);
uint8_t restore_flight_data_from_backup_sram(void);

// Backup SRAM base address (STM32F446 has 4KB backup SRAM at 0x40024000)
#define BACKUP_SRAM_BASE_ADDRESS 0x40024000
#define BACKUP_SRAM_DATA_ADDRESS (BACKUP_SRAM_BASE_ADDRESS)

#endif /* INC_RESET_DETECT_H_ */
