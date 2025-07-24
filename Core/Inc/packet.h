/*
 * packet.h
 *
 *  Created on: May 7, 2025
 *      Author: Halil
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include "bmi088.h"
#include "bme280.h"


typedef union {
    float sayi;
    uint8_t array[4];
} FLOAT32_UINT8_DONUSTURUCU;




extern float sut_altitude, sut_pressure, sut_acc_x, sut_acc_y, sut_acc_z;
extern float sut_gyro_x, sut_gyro_y, sut_gyro_z;

unsigned char check_sum_hesapla_normal(int a);
unsigned char check_sum_hesapla_sit(int a);


void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI);
void addDataPacketSit(BME_280_t* BME, bmi088_struct_t* BMI);

float uint8_arrayi_float32_ye_donustur(uint8_t byte_array_u8[4]);
void verileri_coz(uint8_t gelen_paket[36]);

#endif /* INC_PACKET_H_ */
