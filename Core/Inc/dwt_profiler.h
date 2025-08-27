/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwt_profiler.h
  * @brief          : DWT Performance Profiler Header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
  ******************************************************************************
  * DWT Performance Profiler
  * 
  * This module provides high-precision performance profiling using the ARM
  * Data Watchpoint and Trace (DWT) unit for STM32 microcontrollers.
  * 
  * Features:
  * - CPU cycle-accurate timing measurements
  * - Automatic profiling ID management
  * - Statistical analysis (min, max, average)
  * - Easy-to-use macros for code instrumentation
  * - UART output formatting
  * 
  * Usage:
  * 1. Call dwt_profiler_init() during system initialization
  * 2. Use PROFILE_START(id) and PROFILE_END(id) macros around code sections
  * 3. Call dwt_profiler_print_results() periodically to output results
  * 
  ******************************************************************************
  */

#ifndef __DWT_PROFILER_H__
#define __DWT_PROFILER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define DWT_PROFILER_MAX_PROFILES   16  // Maximum number of concurrent profiles


// DWT (Data Watchpoint and Trace) için makrolar - hassas timing ölçümü
#define DWT_CYCCNT_ENABLED      (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)
#define DWT_CYCCNT              (DWT->CYCCNT)
#define DWT_ENABLE_CYCLE_COUNT() \
    do { \
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
        DWT->CYCCNT = 0; \
    } while(0)

// Mikrosaniye hesaplama makrosu (84MHz sistem saati)
#define CYCLES_TO_MICROSECONDS(cycles) ((cycles) / (SystemCoreClock / 1000000))
#define CYCLES_TO_MILLISECONDS(cycles) ((cycles) / (SystemCoreClock / 1000))

/* Profile IDs - Add new IDs here */
typedef enum {
    PROF_BMI088_UPDATE = 0,
    PROF_BME280_UPDATE,
	PROF_TEST,
    PROF_ADC_READ,
    PROF_SENSOR_FUSION,
    PROF_FLIGHT_ALGORITHM,
    PROF_GNSS_UPDATE,
    PROF_PACKET_COMPOSE,
    PROF_UART2_SEND,
    PROF_LORA_SEND,
	PROF_SD_LOGGER,
	PROF_FLASH,
    PROF_MAIN_LOOP,
    // Add more profile IDs here as needed
    PROF_MAX_COUNT
} dwt_profile_id_t;

/* Profile statistics structure */
typedef struct {
    uint32_t total_cycles;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint32_t count;
    uint32_t last_cycles;
    const char* name;
    bool active;
    uint32_t start_time;
} dwt_profile_t;

/* Exported macros -----------------------------------------------------------*/
#define PROFILE_START(id)   dwt_profiler_start(id)
#define PROFILE_END(id)     dwt_profiler_end(id)

// Convenience macros for one-shot measurements
#define PROFILE_BLOCK_START(id, name) \
    do { \
        dwt_profiler_set_name(id, name); \
        PROFILE_START(id); \
    } while(0)

#define PROFILE_BLOCK_END(id) PROFILE_END(id)

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize DWT profiler
 * @note Must be called before using any profiler functions
 * @retval true if initialization successful, false otherwise
 */
bool dwt_profiler_init(void);

/**
 * @brief Start profiling a code section
 * @param id Profile ID
 * @retval None
 */
void dwt_profiler_start(dwt_profile_id_t id);

/**
 * @brief End profiling a code section
 * @param id Profile ID
 * @retval None
 */
void dwt_profiler_end(dwt_profile_id_t id);

/**
 * @brief Set custom name for a profile ID
 * @param id Profile ID
 * @param name Profile name string
 * @retval None
 */
void dwt_profiler_set_name(dwt_profile_id_t id, const char* name);

/**
 * @brief Reset all profiling statistics
 * @retval None
 */
void dwt_profiler_reset(void);

/**
 * @brief Reset specific profile statistics
 * @param id Profile ID
 * @retval None
 */
void dwt_profiler_reset_profile(dwt_profile_id_t id);

/**
 * @brief Get profiling results for specific ID
 * @param id Profile ID
 * @retval Pointer to profile structure or NULL if invalid ID
 */
const dwt_profile_t* dwt_profiler_get_profile(dwt_profile_id_t id);

/**
 * @brief Convert cycles to microseconds
 * @param cycles CPU cycles
 * @retval Microseconds
 */
uint32_t dwt_profiler_cycles_to_us(uint32_t cycles);

/**
 * @brief Convert cycles to milliseconds
 * @param cycles CPU cycles
 * @retval Milliseconds
 */
uint32_t dwt_profiler_cycles_to_ms(uint32_t cycles);

/**
 * @brief Get average execution time in microseconds
 * @param id Profile ID
 * @retval Average execution time in microseconds
 */
uint32_t dwt_profiler_get_avg_us(dwt_profile_id_t id);

/**
 * @brief Print all profiling results via UART
 * @param uart_send_func Function pointer to UART send function
 * @retval None
 */
void dwt_profiler_print_results();

/**
 * @brief Print compact profiling results
 * @param uart_send_func Function pointer to UART send function
 * @retval None
 */
void dwt_profiler_print_compact();

/**
 * @brief Get current DWT cycle counter value
 * @retval Current cycle count
 */
static inline uint32_t dwt_profiler_get_cycles(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief Check if DWT is enabled and running
 * @retval true if DWT is enabled, false otherwise
 */
bool dwt_profiler_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __DWT_PROFILER_H__ */
