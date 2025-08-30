/*
 * reset_detect.c
 *
 *  Created on: Aug 22, 2024
 *      Author: yahya
 */

#include "reset_detect.h"
#include "string.h"

// Global backup SRAM data pointer
static backup_sram_data_t* backup_data = (backup_sram_data_t*)BACKUP_SRAM_DATA_ADDRESS;

/**
 * @brief Initialize backup SRAM
 */
void backup_sram_init(void)
{
    // Enable backup SRAM clock
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    
    // Enable backup domain access
    HAL_PWR_EnableBkUpAccess();
    
    // Enable backup SRAM regulator
    HAL_PWREx_EnableBkUpReg();
    
    // Check if backup SRAM contains valid data
    if (backup_data->magic_number != BACKUP_SRAM_MAGIC_NUMBER) {
        // Initialize backup SRAM with default values
        memset((void*)backup_data, 0, sizeof(backup_sram_data_t));
        backup_data->magic_number = BACKUP_SRAM_MAGIC_NUMBER;
        backup_data->last_save_timestamp = 0;
    }
}

/**
 * @brief Save current time to RTC backup register
 */
void save_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t)
{
	uint32_t seconds = 0;
	seconds += time_t.Hours * 3600 + time_t.Minutes * 60 + time_t.Seconds;
	seconds += date_t.Date * 86400;
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, seconds);
}

/**
 * @brief Calculate absolute time difference
 */
uint32_t measure_abs_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t)
{
	uint32_t seconds = 0;
	seconds += time_t.Hours * 3600 + time_t.Minutes * 60 + time_t.Seconds;
	seconds += date_t.Date * 86400;
	HAL_PWR_EnableBkUpAccess();
	uint32_t saved_seconds = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	return((uint32_t)abs((int32_t)seconds - (int32_t)saved_seconds));
}

/**
 * @brief Check if reset occurred by comparing current time with saved time
 * @param time_t Current RTC time
 * @param date_t Current RTC date
 * @return 1 if reset occurred, 0 if normal operation
 */
uint8_t is_reset_occurred(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t)
{
    // If time difference is greater than 3 seconds, reset occurred
    if (measure_abs_time(time_t, date_t) > 1) {
        return 0; // No reset
    }
    return 1; // Reset occurred
}

/**
 * @brief Save static calibration data to backup SRAM (called only at startup)
 * @param bme Pointer to BME280 structure
 * @param bmi Pointer to BMI088 structure
 */
void save_static_calibration_data(BME_280_t* bme, bmi088_struct_t* bmi)
{
    if (!bme || !bmi) return;
    
    // Save BME280 calibration parameters (static, calculated once)
    if (bme->parameters) {
        backup_data->bme_params = *(bme->parameters);
    }
    
    // Save BMI088 offset data (static, calculated once)
    if (bmi->device_config.offsets) {
        backup_data->bmi_offsets = *(bmi->device_config.offsets);
    }
    
    // Save base altitude (static, calculated once)
    backup_data->base_altitude = bme->base_altitude;
    
    // Update timestamp
    backup_data->last_save_timestamp = HAL_GetTick();
}

/**
 * @brief Restore critical sensor data from backup SRAM
 * @param bme Pointer to BME280 structure
 * @param bmi Pointer to BMI088 structure
 * @return 1 if data restored successfully, 0 if failed
 */
uint8_t restore_critical_data_from_backup_sram(BME_280_t* bme, bmi088_struct_t* bmi)
{
    if (!bme || !bmi) return 0;
    
    // Check if backup data is valid
    if (backup_data->magic_number != BACKUP_SRAM_MAGIC_NUMBER) {
        return 0; // Invalid data
    }
    
    // Restore BME280 calibration parameters
    if (bme->parameters) {
        *(bme->parameters) = backup_data->bme_params;
    }
    
    // Restore BMI088 offset data  
    if (bmi->device_config.offsets) {
        *(bmi->device_config.offsets) = backup_data->bmi_offsets;
    }
    
    // Restore base altitude
    bme->base_altitude = backup_data->base_altitude;
    
    return 1; // Success
}

/**
 * @brief Save flight algorithm critical data to backup SRAM
 */
void save_flight_data_to_backup_sram(void)
{
    // Get current flight data
    backup_data->flight_data.current_phase = flight_algorithm_get_phase();
    backup_data->flight_data.status_bits = flight_algorithm_get_status_bits();
    backup_data->flight_data.durum_verisi = flight_algorithm_get_durum_verisi();
    backup_data->flight_data.flight_start_time = flight_algorithm_get_start_time();
    
    // Update timestamp
    backup_data->last_save_timestamp = HAL_GetTick();
}

/**
 * @brief Restore flight algorithm critical data from backup SRAM
 * @return 1 if data restored successfully, 0 if failed
 */
uint8_t restore_flight_data_from_backup_sram(void)
{
    // Check if backup data is valid
    if (backup_data->magic_number != BACKUP_SRAM_MAGIC_NUMBER) {
        return 0; // Invalid data
    }
    
    // Restore flight algorithm state using the restore function
    flight_algorithm_restore_state(
        backup_data->flight_data.current_phase,
        backup_data->flight_data.status_bits,
        backup_data->flight_data.durum_verisi,
        backup_data->flight_data.flight_start_time
    );
    
    return 1; // Success
}
