/**
 * @file sensor_fusion.c
 * @brief Sensor fusion implementation that uses external Kalman filter
 * @date 2025-07-05 13:48:29
 * @author halilsrky
 */

#include "sensor_fusion.h"
#include "kalman.h"
#include "queternion.h"
#include "quaternion.h"
#include "flight_algorithm.h"  // Flight algorithm için include ekledik
#include <math.h>

/* Private variables */
static KalmanFilter_t kalman;          // Kalman filter instance from kalman.h
static uint8_t initialized = 0;        // Flag to track initialization
static float reference_altitude = 0.0f; // Base reference altitude
static uint32_t last_kalman_update_time = 0;  // Timestamp of last Kalman update
static uint32_t flight_start_time = 0;  // Timestamp when flight began (from flight algorithm)

/* İvme sensörü arıza tespiti için değişkenler */
#define ACCEL_BUFFER_SIZE 5            // İvme değerlerini saklamak için buffer boyutu
static float accel_buffer[ACCEL_BUFFER_SIZE]; // İvme değerleri için halka tampon
static uint8_t accel_buffer_index = 0;        // Tampon indeksi
static uint8_t accel_buffer_full = 0;         // Tampon doldu mu?
static uint8_t accel_failure_detected = 0;    // Arıza tespit edildi mi?

/* Dinamik arıza tespit eşik değerleri */
#define THRUST_PHASE_DURATION_MS 15000  // İtki fazı süresi (15 saniye)
#define ACCEL_MAX_VALUE_THRUST 150.0f   // İtki fazında max ivme (m/s²)
#define ACCEL_MAX_STD_THRUST 50.0f      // İtki fazında max std sapma
#define ACCEL_MAX_VALUE_CRUISE 50.0f    // Seyir fazında max ivme
#define ACCEL_MAX_STD_CRUISE 15.0f      // Seyir fazında max std sapma

/* External variables from main */
extern uint8_t tx_timer_flag;  // Timer flag that is set every 100ms

/**
 * @brief İvme değerlerinin standart sapmasını hesapla
 * @return Standart sapma değeri
 */
static float calculate_accel_std_deviation(void)
{
    // Buffer dolmadıysa ve çok az veri varsa
    if (!accel_buffer_full && accel_buffer_index < 2) return 0.0f;

    int count = accel_buffer_full ? ACCEL_BUFFER_SIZE : accel_buffer_index;
    float sum = 0.0f;
    float mean = 0.0f;
    float variance = 0.0f;

    // Ortalama hesapla
    for (int i = 0; i < count; i++) {
        sum += accel_buffer[i];
    }
    mean = sum / count;

    // Varyans hesapla
    for (int i = 0; i < count; i++) {
        variance += (accel_buffer[i] - mean) * (accel_buffer[i] - mean);
    }
    variance /= count;

    return sqrtf(variance);
}

/**
 * @brief İvme sensörünün arızalı olup olmadığını kontrol et
 * @param accel İvme değeri (m/s²)
 * @return 1: Arıza tespit edildi, 0: Arıza yok
 */
static uint8_t detect_accel_failure(float accel)
{
    float accel_abs = fabsf(accel);
    float max_accel, max_std;

    // Buffer güncelleme
    accel_buffer[accel_buffer_index] = accel_abs;
    accel_buffer_index = (accel_buffer_index + 1) % ACCEL_BUFFER_SIZE;
    if (accel_buffer_index == 0) {
        accel_buffer_full = 1;
    }

    // Standart sapma hesapla
    float std_dev = calculate_accel_std_deviation();

    // Uçuş başlangıç zamanını flight_algorithm'dan al
	uint32_t algorithm_start_time = flight_algorithm_get_start_time();

	// Uçuş başlamışsa flight_algorithm'dan gelen zamanı kullan
	if (algorithm_start_time > 0) {
		flight_start_time = algorithm_start_time;
	}

	// Uçuş fazına göre limit değerlerini belirle
	if (flight_start_time == 0) {
		max_accel = 200.0f;  // Yer hazırlığında gerçekçi olmayan çok yüksek değer
		max_std = 100.0f;    // Yer hazırlığında çok yüksek standart sapma eşiği
	}
	else {
		// *** UÇUŞ BAŞLADI - NORMAL ARIZA TESPİTİ ***
		uint32_t flight_elapsed_time = HAL_GetTick() - flight_start_time;

		if (flight_elapsed_time < THRUST_PHASE_DURATION_MS) {
			// İtki fazı - Yüksek limitler
			max_accel = ACCEL_MAX_VALUE_THRUST;
			max_std = ACCEL_MAX_STD_THRUST;
		} else {
			// Seyir fazı - Düşük limitler
			max_accel = ACCEL_MAX_VALUE_CRUISE;
			max_std = ACCEL_MAX_STD_CRUISE;
		}
	}

    // İvme değeri veya standart sapma limitler dışındaysa
    if (accel_abs > max_accel || std_dev > max_std) {
        return 1;  // Arıza tespit edildi
    }

    return 0;  // Arıza yok
}

/**
 * @brief Initialize the sensor fusion module
 */
void sensor_fusion_init(BME_280_t* BME)
{
    KalmanFilter_Init(&kalman);

    // Sensörlerinize göre gürültü değerlerini ayarlayın
    kalman.process_noise = 0.1f;         // Model gürültüsü
    kalman.measurement_noise_alt = 0.005f;  // BME280 yükseklik gürültüsü
    kalman.measurement_noise_acc = 5.0f;  // BMI088 ivme gürültüsü


    // İvme arıza tespit değişkenlerini sıfırla
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
        accel_buffer[i] = 0.0f;
    }
    accel_buffer_index = 0;
    accel_buffer_full = 0;
    accel_failure_detected = 0;

    initialized = 1;
    reference_altitude = BME->base_altitude;
    last_kalman_update_time = HAL_GetTick();
    flight_start_time = 0;
}

/**
 * @brief Update sensor fusion with new measurements (Kalman filter)
 */
void sensor_fusion_update_kalman(BME_280_t* BME, bmi088_struct_t* BMI, sensor_fusion_t* sensor)
{
    // Get current time for automatic delta calculation
    uint32_t current_time = HAL_GetTick();

    // Calculate time difference in seconds
    float time_sec = (current_time - last_kalman_update_time) / 1000.0f;

    // Update the last update time
    last_kalman_update_time = current_time;

    float altitude = BME->altitude;

    // Ensure time is valid (never zero or negative)
    if (time_sec <= 0.001f) {
        time_sec = 0.01f; // Use default time step if invalid
    }

    // Calculate vertical acceleration by compensating for gravity using IMU orientation
    float angle_rad = BMI->datas.theta * (M_PI / 180.0f);  // dereceyse radyana çevir

    // Yerçekimi ivmesinin lokal z eksenindeki bileşeni
    float g_local_z = 9.81f * cos(angle_rad);

    // Gerçek ivmeyi hesapla:
    float accel_z_corrected = BMI->datas.acc_z - g_local_z;

    // İvme sensörü arıza tespiti
    accel_failure_detected = detect_accel_failure(accel_z_corrected);

    // Arıza durumuna göre Kalman filtresi parametrelerini güncelle
    if (accel_failure_detected) {
        // Arıza tespit edildi - ivme sensörüne çok az güven
        kalman.measurement_noise_acc = 50.0f;
    } else {
        // Normal durum - normal güven
        kalman.measurement_noise_acc = 0.3f;
    }

    // Only update if initialized
    if (initialized) {
        sensor->filtered_altitude = KalmanFilter_Update(&kalman, altitude, accel_z_corrected, time_sec);
        sensor->apogeeDetect = KalmanFilter_IsApogeeDetected(&kalman);
        sensor->velocity = Kalman_Get_Velocity(&kalman);
        // Store velocity estimate for later use
     /*   static float prev_altitude = 0.0f;
        if (time_sec > 0.001f) {
            sensor->velocity = (sensor->filtered_altitude - prev_altitude) / time_sec;
        }
        prev_altitude = sensor->filtered_altitude;*/

        // Arıza durumunu sensor yapısına ekleyelim (telemetri için kullanılabilir)
        sensor->accel_failure = accel_failure_detected;
    }
}

/**
 * @brief Update orientation using Mahony filter
 */
void sensor_fusion_update_mahony(bmi088_struct_t* BMI, sensor_fusion_t *sensor)
{
	/*if(BMI->isUpdated){
		Orientation_Update(BMI->gyro_x, BMI->gyro_y, BMI->gyro_z,BMI->acc_x, BMI->acc_y, BMI->acc_z, BMI->deltaTime);
		sensor->angle = quaternionToTheta();
		sensor->yaw = quaternionToYaw();
		sensor->pitch = quaternionToPitch();
		sensor->roll = quaternionToRoll();
		BMI->isUpdated = 0;
	}*/
}
