
/**
 * @file sensor_fusion.c
 * @brief Sensor fusion implementation that uses external Kalman filter
 * @date 2025-07-05
 * @author halilsrky
 */
#include "sensor_fusion.h"
#include "kalman.h"
#include "queternion.h"
#include "quaternion.h"
#include "flight_algorithm.h"
#include <math.h>

// M_PI tanımı yoksa tanımla
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private variables */
static KalmanFilter_t kalman;          // Kalman filter instance from kalman.h
static uint8_t initialized = 0;        // Flag to track initialization
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
#define ACCEL_MAX_VALUE_THRUST 250.0f   // İtki fazında max ivme (m/s²)
#define ACCEL_MAX_STD_THRUST 100.0f      // İtki fazında max std sapma
#define ACCEL_MAX_VALUE_CRUISE 100.0f    // Seyir fazında max ivme
#define ACCEL_MAX_STD_CRUISE 50.0f      // Seyir fazında max std sapma

/* Yükseklik arıza tespiti için değişkenler */
#define ALT_BUFFER_SIZE 5
static float alt_buffer[ALT_BUFFER_SIZE];
static uint8_t alt_buffer_index = 0;
static uint8_t alt_buffer_full = 0;
static uint8_t altitude_failure_detected = 0;

#define ALT_MAX_VALUE 10000.0f   // Maksimum yükseklik (m)
#define ALT_MIN_VALUE -100.0f    // Minimum yükseklik (m)
#define ALT_MAX_STD 100.0f       // Maksimum ani değişim (standart sapma)

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

static float calculate_alt_std_deviation(void)
{
    if (!alt_buffer_full && alt_buffer_index < 2) return 0.0f;
    int count = alt_buffer_full ? ALT_BUFFER_SIZE : alt_buffer_index;
    float sum = 0.0f, mean = 0.0f, variance = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += alt_buffer[i];
    }
    mean = sum / count;
    for (int i = 0; i < count; i++) {
        variance += (alt_buffer[i] - mean) * (alt_buffer[i] - mean);
    }
    variance /= count;
    return sqrtf(variance);
}

static uint8_t detect_altitude_failure(float altitude)
{
    alt_buffer[alt_buffer_index] = altitude;
    alt_buffer_index = (alt_buffer_index + 1) % ALT_BUFFER_SIZE;
    if (alt_buffer_index == 0) {
        alt_buffer_full = 1;
    }
    float std_dev = calculate_alt_std_deviation();
    if (altitude > ALT_MAX_VALUE || altitude < ALT_MIN_VALUE || std_dev > ALT_MAX_STD) {
        return 1;
    }
    return 0;
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
void sensor_fusion_init()
{
    // Sensörlerinize göre gürültü değerlerini ayarlayın - daha konservatif başlangıç
    kalman.process_noise = 0.01f;         // Model gürültüsü
    kalman.measurement_noise_alt = 0.01f;  // BME280 yükseklik gürültüsü
    kalman.measurement_noise_acc = 0.5f;  // BMI088 ivme gürültüsü

    KalmanFilter_Init(&kalman);

    // İvme arıza tespit değişkenlerini sıfırla
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
        accel_buffer[i] = 0.0f;
    }
    accel_buffer_index = 0;
    accel_buffer_full = 0;
    accel_failure_detected = 0;

    // Yükseklik arıza tespit değişkenlerini sıfırla
    for (int i = 0; i < ALT_BUFFER_SIZE; i++) {
        alt_buffer[i] = 0.0f;
    }
    alt_buffer_index = 0;
    alt_buffer_full = 0;
    altitude_failure_detected = 0;

    initialized = 1;
    last_kalman_update_time = 0;  // Sıfırla ki ilk çalıştırmada düzgün başlasın
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
    float time_sec;
    
    // İlk çalıştırmada zamanı başlat
    if (last_kalman_update_time == 0) {
        last_kalman_update_time = current_time;
        time_sec = 0.01f; // İlk iterasyon için sabit değer
    } else {
        // Zaman farkını hesapla
        uint32_t time_diff = current_time - last_kalman_update_time;
        time_sec = time_diff / 1000.0f;
        
        // Zaman kontrolü - makul sınırlar içinde olmalı
        if (time_sec <= 0.0f || time_sec > 1.0f) {
            time_sec = 0.01f; // Default 10ms if invalid
        }
    }

    // Update the last update time
    last_kalman_update_time = current_time;

    // Güvenlik kontrolü - BME ve BMI sensör verilerinin geçerli olduğundan emin ol
    if (BME == NULL || BMI == NULL || sensor == NULL) {
        return;
    }

    float t = BMI->datas.theta * (M_PI / 180.0f);  // dereceyse radyana çevir
     // Gravity'in body-z bileşeni
    float g_b_z = 9.80665 * cosf(t);

     // Body-frame gerçek z-ivmesi (yerçekimi çıkarıldı)
    float a_body_z = (-BMI->datas.acc_x) - g_b_z;

     // İnertial dikey ivme yaklaşık olarak:
    float accel_z_corrected = a_body_z * cosf(t);


    // İvme sensörü arıza tespiti
    accel_failure_detected = detect_accel_failure(accel_z_corrected);

    // Yükseklik arıza tespiti
    altitude_failure_detected = detect_altitude_failure(BME->altitude);

    // İvme değeri kontrolü - aşırı değerleri sınırla
    if (fabsf(accel_z_corrected) > 500.0f) {
        accel_z_corrected = (accel_z_corrected > 0) ? 500.0f : -500.0f;
    }


    if (initialized) {
        // Arızalı sensör verileriyle bile sınırlı işlem yap
        if (accel_failure_detected && !altitude_failure_detected) {
            // Sadece yükseklik verisi kullan, ivme verisine düşük ağırlık ver
            kalman.measurement_noise_acc = 5000.5f;
            kalman.measurement_noise_alt = 0.005f;
            sensor->filtered_altitude = KalmanFilter_Update(&kalman, BME->altitude, 0.0f, time_sec);
            sensor->velocity = Kalman_Get_Velocity(&kalman);
            sensor->apogeeDetect = KalmanFilter_IsApogeeDetected(&kalman);
        }
        else if (!accel_failure_detected && altitude_failure_detected) {
            // Sadece ivme verisi kullan, yüksekliğe düşük ağırlık ver
            kalman.measurement_noise_alt = 5000.5f;
            kalman.measurement_noise_acc = 0.005f;
            sensor->filtered_altitude = KalmanFilter_Update(&kalman, BME->altitude, accel_z_corrected, time_sec);
            sensor->velocity = Kalman_Get_Velocity(&kalman);
            sensor->apogeeDetect = KalmanFilter_IsApogeeDetected(&kalman);
        }
        else if (!accel_failure_detected && !altitude_failure_detected) {
            kalman.measurement_noise_alt = 0.01f;
            kalman.measurement_noise_acc = 0.5f;
            sensor->filtered_altitude = KalmanFilter_Update(&kalman, BME->altitude, accel_z_corrected, time_sec);
            sensor->velocity = Kalman_Get_Velocity(&kalman);
            sensor->apogeeDetect = KalmanFilter_IsApogeeDetected(&kalman);
        }
        else {
            // Her iki sensör de arızalı - son değerleri koru ve hafifçe tahmin et
            // Hiçbir sensör verisi kullanma, sadece tahmine devam et
            float saved_alt_noise = kalman.measurement_noise_alt;
            float saved_acc_noise = kalman.measurement_noise_acc;

            // Ölçümlere güvenme, sadece model tahminini kullan
            kalman.measurement_noise_alt = 10000.0f;
            kalman.measurement_noise_acc = 10000.0f;

            sensor->filtered_altitude = KalmanFilter_Update(&kalman, BME->altitude, 0.0f, time_sec);
            sensor->velocity = Kalman_Get_Velocity(&kalman);
            sensor->apogeeDetect = KalmanFilter_IsApogeeDetected(&kalman);

            // Orijinal gürültü parametrelerini geri yükle
            kalman.measurement_noise_alt = saved_alt_noise;
            kalman.measurement_noise_acc = saved_acc_noise;
        }
    }
    
    // Her durumda arıza flaglerini güncelle
    sensor->accel_failure = accel_failure_detected;
    sensor->altitude_failure = altitude_failure_detected;
}
