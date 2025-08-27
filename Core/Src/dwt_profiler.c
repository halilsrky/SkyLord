/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwt_profiler.c
  * @brief          : DWT Performance Profiler Implementation
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

/* Includes ------------------------------------------------------------------*/
#include "dwt_profiler.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk      (1UL << 0)
#endif

#ifndef CoreDebug_DEMCR_TRCENA_Msk
#define CoreDebug_DEMCR_TRCENA_Msk  (1UL << 24)
#endif

extern UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
static dwt_profile_t profiles[DWT_PROFILER_MAX_PROFILES];
static bool profiler_initialized = false;

/* Default profile names */
static const char* default_names[PROF_MAX_COUNT] = {
    "BMI088",
    "BME280",
	"TEST",
    "ADC",
    "FUSION",
    "FLIGHT",
    "GNSS",
    "PACKET",
    "UART2",
    "LORA",
	"LOGGER",
	"FLASH",
    "MAIN_LOOP"
};

/* Private function prototypes -----------------------------------------------*/
static bool dwt_enable_cycle_counter(void);
static void dwt_reset_profile_stats(dwt_profile_id_t id);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize DWT profiler
 */
bool dwt_profiler_init(void)
{
    // Enable DWT cycle counter
    if (!dwt_enable_cycle_counter()) {
        return false;
    }
    
    // Initialize all profiles
    memset(profiles, 0, sizeof(profiles));
    
    // Set default names
    for (int i = 0; i < PROF_MAX_COUNT && i < DWT_PROFILER_MAX_PROFILES; i++) {
        profiles[i].name = default_names[i];
        profiles[i].min_cycles = UINT32_MAX;
        profiles[i].max_cycles = 0;
        profiles[i].active = false;
    }
    
    profiler_initialized = true;
    return true;
}

/**
 * @brief Start profiling a code section
 */
void dwt_profiler_start(dwt_profile_id_t id)
{
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES) {
        return;
    }
    
    profiles[id].start_time = DWT->CYCCNT;
    profiles[id].active = true;
}

/**
 * @brief End profiling a code section
 */
void dwt_profiler_end(dwt_profile_id_t id)
{
    uint32_t end_time = DWT->CYCCNT;
    
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES || !profiles[id].active) {
        return;
    }
    
    // Calculate elapsed cycles (handle counter overflow)
    uint32_t elapsed_cycles;
    if (end_time >= profiles[id].start_time) {
        elapsed_cycles = end_time - profiles[id].start_time;
    } else {
        // Counter overflow occurred
        elapsed_cycles = (UINT32_MAX - profiles[id].start_time) + end_time + 1;
    }
    
    // Update statistics
    profiles[id].last_cycles = elapsed_cycles;
    profiles[id].total_cycles += elapsed_cycles;
    profiles[id].count++;
    
    if (elapsed_cycles < profiles[id].min_cycles) {
        profiles[id].min_cycles = elapsed_cycles;
    }
    
    if (elapsed_cycles > profiles[id].max_cycles) {
        profiles[id].max_cycles = elapsed_cycles;
    }
    
    profiles[id].active = false;
}

/**
 * @brief Set custom name for a profile ID
 */
void dwt_profiler_set_name(dwt_profile_id_t id, const char* name)
{
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES || !name) {
        return;
    }
    
    profiles[id].name = name;
}

/**
 * @brief Reset all profiling statistics
 */
void dwt_profiler_reset(void)
{
    if (!profiler_initialized) {
        return;
    }
    
    for (int i = 0; i < DWT_PROFILER_MAX_PROFILES; i++) {
        dwt_reset_profile_stats(i);
    }
}

/**
 * @brief Reset specific profile statistics
 */
void dwt_profiler_reset_profile(dwt_profile_id_t id)
{
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES) {
        return;
    }
    
    dwt_reset_profile_stats(id);
}

/**
 * @brief Get profiling results for specific ID
 */
const dwt_profile_t* dwt_profiler_get_profile(dwt_profile_id_t id)
{
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES) {
        return NULL;
    }
    
    return &profiles[id];
}

/**
 * @brief Convert cycles to microseconds
 */
uint32_t dwt_profiler_cycles_to_us(uint32_t cycles)
{
    // Assuming SystemCoreClock is available
    extern uint32_t SystemCoreClock;
    return cycles / (SystemCoreClock / 1000000);
}

/**
 * @brief Convert cycles to milliseconds
 */
uint32_t dwt_profiler_cycles_to_ms(uint32_t cycles)
{
    extern uint32_t SystemCoreClock;
    return cycles / (SystemCoreClock / 1000);
}

/**
 * @brief Get average execution time in microseconds
 */
uint32_t dwt_profiler_get_avg_us(dwt_profile_id_t id)
{
    if (!profiler_initialized || id >= DWT_PROFILER_MAX_PROFILES || profiles[id].count == 0) {
        return 0;
    }
    
    uint32_t avg_cycles = profiles[id].total_cycles / profiles[id].count;
    return dwt_profiler_cycles_to_us(avg_cycles);
}

/**
 * @brief Print all profiling results via UART
 */
void dwt_profiler_print_results()
{
    static char buffer[1024];
    
    if (!profiler_initialized) {
        return;
    }
    
    int len = snprintf(buffer, sizeof(buffer), 
        "\r\n=== DWT PROFILER RESULTS ===\r\n");
    
    for (int i = 0; i < DWT_PROFILER_MAX_PROFILES; i++) {
        if (profiles[i].count > 0 && profiles[i].name) {
            uint32_t avg_us = dwt_profiler_get_avg_us(i);
            uint32_t min_us = dwt_profiler_cycles_to_us(profiles[i].min_cycles);
            uint32_t max_us = dwt_profiler_cycles_to_us(profiles[i].max_cycles);
            uint32_t last_us = dwt_profiler_cycles_to_us(profiles[i].last_cycles);
            
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "%s: AVG:%uus MIN:%uus MAX:%uus LAST:%uus COUNT:%u\r\n",
                profiles[i].name, 
                (unsigned int)avg_us,
                (unsigned int)min_us,
                (unsigned int)max_us,
                (unsigned int)last_us,
                (unsigned int)profiles[i].count);
                
            if (len >= sizeof(buffer) - 100) break; // Prevent buffer overflow
        }
    }
    
    len += snprintf(buffer + len, sizeof(buffer) - len, "===========================\r\n");
    
    if (len > 0 && len < sizeof(buffer)) {
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
    }
}

/**
 * @brief Print compact profiling results
 */
void dwt_profiler_print_compact()
{
    static char buffer[512];
    
    if (!profiler_initialized) {
        return;
    }
    
    int len = snprintf(buffer, sizeof(buffer), "PROF(us):");
    
    for (int i = 0; i < DWT_PROFILER_MAX_PROFILES && i < PROF_MAX_COUNT; i++) {
        if (profiles[i].count > 0 && profiles[i].name) {
            uint32_t last_us = dwt_profiler_cycles_to_us(profiles[i].last_cycles);
            
            len += snprintf(buffer + len, sizeof(buffer) - len,
                " %s:%u", profiles[i].name, (unsigned int)last_us);
                
            if (len >= sizeof(buffer) - 50) break; // Prevent buffer overflow
        }
    }
    
    len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
    
    if (len > 0 && len < sizeof(buffer)) {
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);

    }
}

/**
 * @brief Check if DWT is enabled and running
 */
bool dwt_profiler_is_enabled(void)
{
    return profiler_initialized && 
           (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
           (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Enable DWT cycle counter
 */
static bool dwt_enable_cycle_counter(void)
{
    // Enable DWT trace
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // Reset cycle counter
    DWT->CYCCNT = 0;
    
    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // Verify that cycle counter is running
    uint32_t start = DWT->CYCCNT;
    for (volatile int i = 0; i < 100; i++); // Small delay
    uint32_t end = DWT->CYCCNT;
    
    return (end > start);
}

/**
 * @brief Reset profile statistics
 */
static void dwt_reset_profile_stats(dwt_profile_id_t id)
{
    profiles[id].total_cycles = 0;
    profiles[id].min_cycles = UINT32_MAX;
    profiles[id].max_cycles = 0;
    profiles[id].count = 0;
    profiles[id].last_cycles = 0;
    profiles[id].active = false;
}
