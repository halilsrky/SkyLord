/*
 * data_logger.c
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */
#include "data_logger.h"
#include "fatfs.h"
#include "diskio.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "main.h"  // GPIO definitions için

// Buffer için tanımlamalar
#define PACKET_SIZE 50
#define PACKETS_PER_SECTOR 10  // 10 paket = 500 byte (512'den küçük)
#define BUFFER_SIZE (PACKET_SIZE * PACKETS_PER_SECTOR)

static uint8_t packet_buffer[BUFFER_SIZE];
static uint16_t buffer_index = 0;
static uint8_t file_opened = 0;

// External variables from fatfs.c
extern FATFS USERFatFS;
extern FIL USERFile;
extern char USERPath[4];
extern SPI_HandleTypeDef hspi1;  // SPI handle
FRESULT fres; 	//Result after operations

//For file operation functions look at https://elm-chan.org/fsw/ff/00index_e.html

// SPI Test fonksiyonu - SD kart bağlantısını test etmek için
uint8_t test_spi_connection(void)
{
	// CS pin'i test et
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);  // CS LOW
	HAL_Delay(1);
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);    // CS HIGH
	
	// SPI test - basit veri gönder/al
	uint8_t test_data = 0xFF;
	uint8_t received_data = 0x00;
	
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);  // CS LOW
	HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi1, &test_data, &received_data, 1, 100);
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);    // CS HIGH
	
	// SPI çalışıyor mu kontrol et
	if (spi_status == HAL_OK) {
		return 1;  // SPI bağlantısı var
	}
	return 0;  // SPI bağlantısı yok
}

void data_logger_init()
{
	// SPI bağlantısını test et
	uint8_t spi_test = test_spi_connection();
	if (spi_test == 0) {
		// SPI bağlantısı yok - hardware sorunu
		volatile uint8_t debug_spi_error = 1;  // Breakpoint koyun
		return;
	}
	// Önce disk status kontrol et
	DSTATUS initial_status = disk_status(0);
	
	// Disk initialize et
	DSTATUS disk_status_result = disk_initialize(0);  // Drive 0
	if (disk_status_result != RES_OK) {
		// Disk initialize başarısız
		// RES_OK=0, RES_ERROR=1, RES_WRPRT=2, RES_NOTRDY=3, RES_PARERR=4
		// STA_NOINIT=0x01, STA_NODISK=0x02, STA_PROTECT=0x04
		volatile DSTATUS debug_initial_status = initial_status;
		volatile DSTATUS debug_disk_status = disk_status_result;  // Breakpoint koyun
		
		// Birkaç kez daha deneyelim
		HAL_Delay(100);
		disk_status_result = disk_initialize(0);
		if (disk_status_result != RES_OK) {
			return;  // İkinci deneme de başarısız
		}
	}
	
	// SD kartı mount et - USERPath kullan
	fres = f_mount(&USERFatFS, USERPath, 1);
	if (fres != FR_OK) {
		// Mount başarısız - hata işleme
		// Debug: fres değerini breakpoint koyup kontrol edin
		// FR_OK=0, FR_DISK_ERR=1, FR_INT_ERR=2, FR_NOT_READY=3, 
		// FR_NO_FILE=4, FR_NO_PATH=5, FR_INVALID_NAME=6, FR_DENIED=7,
		// FR_EXIST=8, FR_INVALID_OBJECT=9, FR_WRITE_PROTECTED=10,
		// FR_INVALID_DRIVE=11, FR_NOT_ENABLED=12, FR_NO_FILESYSTEM=13
		volatile FRESULT debug_fres = fres;  // Bu satıra breakpoint koyun
		return;
	}
	
	// tracker.csv için başlık oluştur
	fres = f_open(&USERFile, "tracker.csv", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres == FR_OK) {
		f_lseek(&USERFile, f_size(&USERFile));
		unsigned int file_res = 0;
		uint8_t p_data[300];
		sprintf((char*) p_data, (char*)"Time,Altitude (m),Lat,Lon,Altitude pressure (m),Temperature (C),Humidity (%%)\n");
		f_write(&USERFile, (uint8_t*) p_data, strlen((char*)p_data), &file_res);
		f_close(&USERFile);
	}
	
	// packet_data.bin dosyasını oluştur (eğer yoksa)
	fres = f_open(&USERFile, "packet_data.bin", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres == FR_OK) {
		f_close(&USERFile);
	}
	
	// Buffer'ı sıfırla
	buffer_index = 0;
}

void log_datas(float altitude, float lat, float lon, float time, float altitude_pressure, float temperature, float humidity)
{
	fres = f_open(&USERFile, "tracker.csv", FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&USERFile, f_size(&USERFile));
	unsigned int file_res = 0;
	uint8_t p_data[300];
	sprintf((char*) p_data, (char*)"%.0f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f\n", time, altitude, lat, lon,  altitude_pressure, temperature, humidity);
	f_write(&USERFile, (uint8_t*) p_data, strlen((char*)p_data), &file_res);
	f_close(&USERFile);
}

void log_normal_packet_data(unsigned char* packet_data)
{
	// Paketi buffer'a kopyala
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
		fres = f_open(&USERFile, "packet_data.bin", FA_WRITE | FA_OPEN_ALWAYS);
		if (fres == FR_OK) {
			f_lseek(&USERFile, f_size(&USERFile));
			unsigned int file_res = 0;
			
			// Buffer'daki tüm veriyi yaz
			f_write(&USERFile, packet_buffer, buffer_index, &file_res);
			f_close(&USERFile);
			
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
