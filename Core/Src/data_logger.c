/*
 * data_logger.c
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */
#include "data_logger.h"
#include "fatfs.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal.h"  // HAL_GetTick() için

// Buffer için tanımlamalar
#define PACKET_SIZE 62
#define HEADER_SIZE 1     // 1 byte header
#define TIMESTAMP_SIZE 4  // 4 byte timestamp (uint32_t)
#define PACKET_WITH_HEADER_TIMESTAMP_SIZE (HEADER_SIZE + TIMESTAMP_SIZE + PACKET_SIZE)
#define PACKETS_PER_SECTOR 7  // 9 paket = 495 byte (512'den küçük)
#define BUFFER_SIZE (PACKET_WITH_HEADER_TIMESTAMP_SIZE * PACKETS_PER_SECTOR)

// Header değerleri
#define PACKET_HEADER 0xAA  // Paket başlangıcını belirten header

static uint8_t packet_buffer[BUFFER_SIZE];
static uint16_t buffer_index = 0;

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; 	//Result after operations

//For file operation functions look at https://elm-chan.org/fsw/ff/00index_e.html

void data_logger_init()
{
	// SD kartı mount et
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		// Mount başarısız - hata işleme
		return;
	}
	
	// packets.bin dosyasını oluştur (eğer yoksa)
	fres = f_open(&fil, "packets.bin", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres == FR_OK) {
		f_close(&fil);
	}
	
	// Buffer'ı sıfırla
	buffer_index = 0;
}

void log_normal_packet_data(unsigned char* packet_data)
{
	// Mevcut timestamp'i al (ms cinsinden)
	uint32_t timestamp = HAL_GetTick();

	// Önce header'ı buffer'a yaz (1 byte)
	packet_buffer[buffer_index] = PACKET_HEADER;
	buffer_index += HEADER_SIZE;

	// Sonra timestamp'i buffer'a yaz (4 byte)
	memcpy(&packet_buffer[buffer_index], &timestamp, TIMESTAMP_SIZE);
	buffer_index += TIMESTAMP_SIZE;

	// Son olarak paketi buffer'a kopyala (50 byte)
	memcpy(&packet_buffer[buffer_index], packet_data, PACKET_SIZE);
	buffer_index += PACKET_SIZE;
	
	// Buffer doldu mu kontrol et
	if (buffer_index >= BUFFER_SIZE) {
		// Buffer'ı SD karta yaz
		flush_packet_buffer();
	}
}

void flush_packet_buffer(void)
{
	if (buffer_index > 0) {
		fres = f_open(&fil, "packets.bin", FA_WRITE | FA_OPEN_ALWAYS);
		if (fres == FR_OK) {
			f_lseek(&fil, f_size(&fil));
			unsigned int file_res = 0;
			
			// Buffer'daki tüm veriyi yaz
			f_write(&fil, packet_buffer, buffer_index, &file_res);
			f_close(&fil);
			
			// Buffer'ı sıfırla
			buffer_index = 0;
		}
	}
}

// Sistem kapanırken veya acil durumda buffer'ı boşalt
void force_flush_buffer(void)
{
	flush_packet_buffer();
}
