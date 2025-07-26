/*
 * packet.c
 *
 *  Created on: May 7, 2025
 *      Author: Halil
 */
#include "packet.h"
#include <math.h>

unsigned char normal_paket[59];
unsigned char sit_paket[36];


unsigned char check_sum_hesapla_normal(int a){
    int check_sum = 0;
    for(int i = 4; i < a; i++){
        check_sum += normal_paket[i];
    }
    return (unsigned char) (check_sum % 256);
}

unsigned char check_sum_hesapla_sit(int a){
    int check_sum = 0;
    for(int i = 0; i < a; i++){
        check_sum += sit_paket[i];
    }
    return (unsigned char) (check_sum % 256);
}

// Float değerleri iki ondalık basamağa yuvarlamak için yardımcı fonksiyon
double round2(double value) {
    return round(value * 100.0) / 100.0;
}

void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI){
  normal_paket[0] = 0xFF; // Sabit
  normal_paket[1] = 0xFF; // Sabit
  normal_paket[2] = 0x54; // Sabit
  normal_paket[3] = 0x52; // Sabit

  normal_paket[4] = 0;   // Takim ID = 0
  normal_paket[5] = 0; // Sayac degeri = 0

  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
  irtifa_float32_uint8_donusturucu.sayi = (float)round2(BME->altitude); // Irtifa degerinin atamasini yapiyoruz.
  normal_paket[6] = irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[7] = irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[8] = irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[9] = irtifa_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU roket_gps_irtifa_float32_uint8_donusturucu;
  roket_gps_irtifa_float32_uint8_donusturucu.sayi = (float)round2(0); // Roket GPS Irtifa degerinin atamasini yapiyoruz.
  normal_paket[10] = roket_gps_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[11] = roket_gps_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[12] = roket_gps_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[13] = roket_gps_irtifa_float32_uint8_donusturucu.array[3];

   // Roket Enlem
  FLOAT32_UINT8_DONUSTURUCU roket_enlem_float32_uint8_donusturucu;
  roket_enlem_float32_uint8_donusturucu.sayi = (float)round2(0); // Roket enlem degerinin atamasini yapiyoruz.
  normal_paket[14] = roket_enlem_float32_uint8_donusturucu.array[0];
  normal_paket[15] = roket_enlem_float32_uint8_donusturucu.array[1];
  normal_paket[16] = roket_enlem_float32_uint8_donusturucu.array[2];
  normal_paket[17] = roket_enlem_float32_uint8_donusturucu.array[3];

  // Roket Boylam
  FLOAT32_UINT8_DONUSTURUCU roket_boylam_irtifa_float32_uint8_donusturucu;
  roket_boylam_irtifa_float32_uint8_donusturucu.sayi = (float)round2(0); // Roket boylam degerinin atamasini yapiyoruz.
  normal_paket[18] = roket_boylam_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[19] = roket_boylam_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[20] = roket_boylam_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[21] = roket_boylam_irtifa_float32_uint8_donusturucu.array[3];


  FLOAT32_UINT8_DONUSTURUCU jiroskop_x_float32_uint8_donusturucu;
  jiroskop_x_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.gyro_x); // Jiroskop X degerinin atamasini yapiyoruz.
  normal_paket[22] = jiroskop_x_float32_uint8_donusturucu.array[0];
  normal_paket[23] = jiroskop_x_float32_uint8_donusturucu.array[1];
  normal_paket[24] = jiroskop_x_float32_uint8_donusturucu.array[2];
  normal_paket[25] = jiroskop_x_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU jiroskop_y_float32_uint8_donusturucu;
  jiroskop_y_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.gyro_y); // Jiroskop Y degerinin atamasini yapiyoruz.
  normal_paket[26] = jiroskop_y_float32_uint8_donusturucu.array[0];
  normal_paket[27] = jiroskop_y_float32_uint8_donusturucu.array[1];
  normal_paket[28] = jiroskop_y_float32_uint8_donusturucu.array[2];
  normal_paket[29] = jiroskop_y_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU jiroskop_z_float32_uint8_donusturucu;
  jiroskop_z_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.gyro_z); // Jiroskop Z degerinin atamasini yapiyoruz.
  normal_paket[30] = jiroskop_z_float32_uint8_donusturucu.array[0];
  normal_paket[31] = jiroskop_z_float32_uint8_donusturucu.array[1];
  normal_paket[32] = jiroskop_z_float32_uint8_donusturucu.array[2];
  normal_paket[33] = jiroskop_z_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU ivme_x_float32_uint8_donusturucu;
  ivme_x_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_x); // Ivme X degerinin atamasini yapiyoruz.
  normal_paket[34] = ivme_x_float32_uint8_donusturucu.array[0];
  normal_paket[35] = ivme_x_float32_uint8_donusturucu.array[1];
  normal_paket[36] = ivme_x_float32_uint8_donusturucu.array[2];
  normal_paket[37] = ivme_x_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU ivme_y_float32_uint8_donusturucu;
  ivme_y_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_y); // Ivme Y degerinin atamasini yapiyoruz.
  normal_paket[38] = ivme_y_float32_uint8_donusturucu.array[0];
  normal_paket[39] = ivme_y_float32_uint8_donusturucu.array[1];
  normal_paket[40] = ivme_y_float32_uint8_donusturucu.array[2];
  normal_paket[41] = ivme_y_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU ivme_z_float32_uint8_donusturucu;
  ivme_z_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_z); // Ivme Z degerinin atamasini yapiyoruz.
  normal_paket[42] = ivme_z_float32_uint8_donusturucu.array[0];
  normal_paket[43] = ivme_z_float32_uint8_donusturucu.array[1];
  normal_paket[44] = ivme_z_float32_uint8_donusturucu.array[2];
  normal_paket[45] = ivme_z_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU aci_float32_uint8_donusturucu;
  aci_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.theta); // Aci degerinin atamasini yapiyoruz.
  normal_paket[46] = aci_float32_uint8_donusturucu.array[0];
  normal_paket[47] = aci_float32_uint8_donusturucu.array[1];
  normal_paket[48] = aci_float32_uint8_donusturucu.array[2];
  normal_paket[49] = aci_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU sicaklik_float32_uint8_donusturucu;
  sicaklik_float32_uint8_donusturucu.sayi = (float)round2(BME->temperature); // Sicaklik degerinin atamasini yapiyoruz.
  normal_paket[50] = sicaklik_float32_uint8_donusturucu.array[0];
  normal_paket[51] = sicaklik_float32_uint8_donusturucu.array[1];
  normal_paket[52] = sicaklik_float32_uint8_donusturucu.array[2];
  normal_paket[53] = sicaklik_float32_uint8_donusturucu.array[3];

  //NEM
  normal_paket[54] = BME->humidity; // Nem degerinin atamasini yapiyoruz

  normal_paket[55] = 0; // Durum bilgisi = Iki parasut de tetiklenmedi

  normal_paket[56] = check_sum_hesapla_normal(56); // Check_sum = check_sum_hesapla();
  normal_paket[57] = 0x0D; // Sabit
  normal_paket[58] = 0x0A;
}

void addDataPacketSit(BME_280_t* BME, bmi088_struct_t* BMI){
  sit_paket[0] = 0xAB; // Sabit

  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
  irtifa_float32_uint8_donusturucu.sayi = (float)round2(BME->base_altitude + BME->altitude); // Irtifa degerinin atamasini yapiyoruz.
  sit_paket[1] = irtifa_float32_uint8_donusturucu.array[3];
  sit_paket[2] = irtifa_float32_uint8_donusturucu.array[2];
  sit_paket[3] = irtifa_float32_uint8_donusturucu.array[1];
  sit_paket[4] = irtifa_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU basinc_float32_uint8_donusturucu;
  basinc_float32_uint8_donusturucu.sayi = (float)round2(BME->pressure); // Roket GPS Irtifa degerinin atamasini yapiyoruz.
  sit_paket[5] = basinc_float32_uint8_donusturucu.array[3];
  sit_paket[6] = basinc_float32_uint8_donusturucu.array[2];
  sit_paket[7] = basinc_float32_uint8_donusturucu.array[1];
  sit_paket[8] = basinc_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU ivme_x_float32_uint8_donusturucu;
  ivme_x_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_x); // Ivme X degerinin atamasini yapiyoruz.
  sit_paket[9] = ivme_x_float32_uint8_donusturucu.array[3];
  sit_paket[10] = ivme_x_float32_uint8_donusturucu.array[2];
  sit_paket[11] = ivme_x_float32_uint8_donusturucu.array[1];
  sit_paket[12] = ivme_x_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU ivme_y_float32_uint8_donusturucu;
  ivme_y_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_y); // Ivme Y degerinin atamasini yapiyoruz.
  sit_paket[13] = ivme_y_float32_uint8_donusturucu.array[3];
  sit_paket[14] = ivme_y_float32_uint8_donusturucu.array[2];
  sit_paket[15] = ivme_y_float32_uint8_donusturucu.array[1];
  sit_paket[16] = ivme_y_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU ivme_z_float32_uint8_donusturucu;
  ivme_z_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.acc_z); // Ivme Z degerinin atamasini yapiyoruz.
  sit_paket[17] = ivme_z_float32_uint8_donusturucu.array[3];
  sit_paket[18] = ivme_z_float32_uint8_donusturucu.array[2];
  sit_paket[19] = ivme_z_float32_uint8_donusturucu.array[1];
  sit_paket[20] = ivme_z_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU jiroskop_x_float32_uint8_donusturucu;
  jiroskop_x_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.angle_x); // Jiroskop X degerinin atamasini yapiyoruz.
  sit_paket[21] = jiroskop_x_float32_uint8_donusturucu.array[3];
  sit_paket[22] = jiroskop_x_float32_uint8_donusturucu.array[2];
  sit_paket[23] = jiroskop_x_float32_uint8_donusturucu.array[1];
  sit_paket[24] = jiroskop_x_float32_uint8_donusturucu.array[0];

  FLOAT32_UINT8_DONUSTURUCU jiroskop_y_float32_uint8_donusturucu;
  jiroskop_y_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.angle_y); // Jiroskop Y degerinin atamasini yapiyoruz.
  sit_paket[25] = jiroskop_y_float32_uint8_donusturucu.array[3];
  sit_paket[26] = jiroskop_y_float32_uint8_donusturucu.array[2];
  sit_paket[27] = jiroskop_y_float32_uint8_donusturucu.array[1];
  sit_paket[28] = jiroskop_y_float32_uint8_donusturucu.array[0];
  FLOAT32_UINT8_DONUSTURUCU jiroskop_z_float32_uint8_donusturucu;
  jiroskop_z_float32_uint8_donusturucu.sayi = (float)round2(BMI->datas.angle_z); // Jiroskop Z degerinin atamasini yapiyoruz.
  sit_paket[29] = jiroskop_z_float32_uint8_donusturucu.array[3];
  sit_paket[30] = jiroskop_z_float32_uint8_donusturucu.array[2];
  sit_paket[31] = jiroskop_z_float32_uint8_donusturucu.array[1];
  sit_paket[32] = jiroskop_z_float32_uint8_donusturucu.array[0];

  sit_paket[33] = check_sum_hesapla_sit(33); // Check_sum = check_sum_hesapla();
  sit_paket[34] = 0x0D;
  sit_paket[35] = 0x0A;

}

float uint8_arrayi_float32_ye_donustur(uint8_t byte_array_u8[4]) {
    FLOAT32_UINT8_DONUSTURUCU float32_uint8_donusturucu;
    float32_uint8_donusturucu.array[0] = byte_array_u8[3];
    float32_uint8_donusturucu.array[1] = byte_array_u8[2];
    float32_uint8_donusturucu.array[2] = byte_array_u8[1];
    float32_uint8_donusturucu.array[3] = byte_array_u8[0];
    return float32_uint8_donusturucu.sayi;
}




