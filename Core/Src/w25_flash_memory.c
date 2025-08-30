/*
 * w25_flash_memory.c
 *
 *  Created on: Aug 24, 2025
 *      Author: bilal
 */

#include "w25_flash_memory.h"

SPI_HandleTypeDef *W25Q_SPI;

volatile uint8_t tx_timer_flag_w25q = 0;
uint8_t sector_erase_started = 0;
uint8_t read_enable = 0;

static uint8_t w25q_txBuffer_1[4096];
static uint8_t w25q_txBuffer_2[4096];
static uint8_t current_buffer = 1;
static uint16_t w25q_page = 0;
static uint8_t non_writen_page = 0;
static uint8_t page_write_started = 0;
static uint8_t w25q_counter = 0;
static uint8_t sector_finished = 0;
static uint8_t write_enable = 0;

static void CS_Low(void);
static void CS_High(void);
static void Write_Enable(void);
static void Write_Disable(void);
static uint16_t Bytes_to_Write(uint32_t remaining_bytes_, uint8_t offset_);
static void Add_to_Buffer(uint8_t *data);
static void W25Q_EraseSector_NonBlocking(uint16_t sector);
static void W25Q_WritePage_NonBlocking(uint16_t page, uint8_t offset, uint32_t size, uint8_t *txBuffer);

void W25Q_Init(SPI_HandleTypeDef *w25q_spi)
{
	W25Q_SPI = w25q_spi;
	W25Q_Reset();
}

void W25Q_Reset(void)
{
	uint8_t command[2];

	command[0] = 0x66;	// Enable reset
	command[1] = 0x99;	// Reset device

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, command, 2, SPI_Timeout);
	CS_High();
	HAL_Delay(50);
}

void W25Q_Read(uint16_t startPage, uint8_t offset, uint32_t size, uint8_t *rxBuffer)
{
	uint32_t memAddress = (startPage * 256) + offset;
	uint8_t command[4];

	command[0] = 0x03;	// Read data
	command[1] = (memAddress >> 16) & 0xFF;
	command[2] = (memAddress >> 8) & 0xFF;
	command[3] = memAddress & 0xFF;

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, command, 4, SPI_Timeout);
	HAL_SPI_Receive(W25Q_SPI, rxBuffer, size, SPI_Timeout);
	CS_High();
}

void W25Q_FastRead(uint16_t startPage, uint8_t offset, uint32_t size, uint8_t *rxBuffer)
{
	uint32_t memAddress = (startPage * 256) + offset;
	uint8_t command[5];

	command[0] = 0x0B;	// Fast read
	command[1] = (memAddress >> 16) & 0xFF;
	command[2] = (memAddress >> 8) & 0xFF;
	command[3] = memAddress & 0xFF;
	command[4] = 0;

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, command, 5, SPI_Timeout);
	HAL_SPI_Receive(W25Q_SPI, rxBuffer, size, SPI_Timeout);
	CS_High();
}

void W25Q_EraseSector(uint16_t sector)
{
	uint32_t memAddress = sector * 16 * 256;
	uint8_t command[4];

	command[0] = 0x20;	// Sector erase
	command[1] = (memAddress >> 16) & 0xFF;
	command[2] = (memAddress >> 8) & 0xFF;
	command[3] = memAddress & 0xFF;

	Write_Enable();
	uint32_t start = HAL_GetTick();
	while((Read_Status_Register_1() & WRITE_ENABLED) == 0)
	{
		if((HAL_GetTick() - start) >= 50)
		{
			break;
		}
	}
	if((Read_Status_Register_1() & WRITE_ENABLED) != 0)
	{
		CS_Low();
		HAL_SPI_Transmit(W25Q_SPI, command, 4, SPI_Timeout);
		CS_High();
	}
/*
	HAL_Delay(450); // 450ms delay for sector erase
	Write_Disable();
*/
}

void W25Q_Write(uint16_t page, uint8_t offset, uint32_t size, uint8_t *txBuffer)
{
	uint8_t data[260];
	uint8_t current_offset = offset;
	uint32_t remaining_bytes = size;
	uint16_t bytes_to_write;
	uint32_t txBuffer_current_index = 0;

	uint16_t startPage = page;
	uint16_t endPage = page + ((offset + size - 1) / 256);
	uint16_t totalPages = endPage - startPage + 1;
/*
	uint16_t startSector = startPage / 16;
	uint16_t endSector = endPage / 16;
	uint16_t totalSector = endSector - startSector + 1;

	for(uint16_t i = 0; i < totalSector; i++)
	{
		W25Q_EraseSector(startSector + i);
	}
*/
	txBuffer_current_index = 0;
	for(uint16_t i = 0; i < totalPages; i++)
	{
		uint32_t memAddress = ((startPage + i) * 256) + current_offset;

		data[0] = 0x02;	// Page program
		data[1] = (memAddress >> 16) & 0xFF;
		data[2] = (memAddress >> 8) & 0xFF;
		data[3] = memAddress & 0XFF;

		bytes_to_write = Bytes_to_Write(remaining_bytes, current_offset);

		for(uint16_t index = 0; index < bytes_to_write; index++)
		{
			data[index + 4] = txBuffer[txBuffer_current_index];
			txBuffer_current_index++;
		}

		Write_Enable();
		if((Read_Status_Register_1() & WRITE_ENABLED) != 0)
		{
			CS_Low();
			HAL_SPI_Transmit(W25Q_SPI, data, (bytes_to_write + 4), SPI_Timeout);
			CS_High();
			uint32_t start = HAL_GetTick();
			while((Read_Status_Register_1() & WIP_BUSY) == 1)
			{
				if((HAL_GetTick() - start) >= 50)
				{
					break;
				}
			}
		}
		Write_Disable();

		current_offset = 0;
		remaining_bytes -= bytes_to_write;
	}
}

void W25Q_WriteToBufferFlushOnSectorFull(uint8_t *data)
{
	// Sadece buffer'a ekle - ultra hızlı (~5μs)
	Add_to_Buffer(data);
	w25q_counter++;

	// Buffer doldu mu?
	if(w25q_counter == 64)
	{
		sector_finished = 1;
		w25q_counter = 0;
		current_buffer = (current_buffer == 1) ? 2 : 1;
	}

	uint8_t status = Read_Status_Register_1();

	// Sector erase başlat (non-blocking)
	if(sector_finished == 1 && sector_erase_started == 0 && write_enable == 0 && (status & WIP_BUSY) == 0)
	{
		W25Q_EraseSector_NonBlocking((w25q_page / 16));
		sector_erase_started = 1;
		w25q_page += 16;
	}

	// Erase tamamlandı mı?
	if(sector_erase_started == 1 && tx_timer_flag_w25q >= 5 && (status & WIP_BUSY) == 0)
	{
		sector_erase_started = 0;
		tx_timer_flag_w25q = 0;
		write_enable = 1;
	}

	static uint8_t counter_ = 0;

	// Write işlemi tamamlandı mı?
	if(page_write_started == 1 && (status & WIP_BUSY) == 0)
	{
		page_write_started = 0;
		Write_Disable();
		counter_++;
		if(counter_ == 16)
		{
			counter_ = 0;
			sector_finished = 0;
			write_enable = 0;
			read_enable = 1;
		}
	}

	// Write işlemi başlat (non-blocking)
	if(sector_finished == 1 && write_enable == 1 && (status & WIP_BUSY) == 0 && page_write_started == 0)
	{
		// Non-blocking burst write
		if(current_buffer == 1)
		{
			W25Q_WritePage_NonBlocking(non_writen_page, 0, 256, &w25q_txBuffer_2[counter_ * 256]);
		}
		else
		{
			W25Q_WritePage_NonBlocking(non_writen_page, 0, 256, &w25q_txBuffer_1[counter_ * 256]);
		}
		non_writen_page++;
		page_write_started = 1;
	}
}

uint8_t Read_Status_Register_1(void)
{
	uint8_t command = 0x05;	// read status register 1
	uint8_t response;

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, &command, 1, SPI_Timeout);
	HAL_SPI_Receive(W25Q_SPI, &response, 1, SPI_Timeout);
	CS_High();

	return response;
}

// ULTRA FAST: Non-blocking sector erase
static void W25Q_EraseSector_NonBlocking(uint16_t sector)
{
	uint32_t memAddress = sector * 16 * 256;
	uint8_t command[4];

	command[0] = 0x20;	// Sector erase
	command[1] = (memAddress >> 16) & 0xFF;
	command[2] = (memAddress >> 8) & 0xFF;
	command[3] = memAddress & 0xFF;

	// Hızlı write enable - delay yok
	uint8_t we_cmd = 0x06;
	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, &we_cmd, 1, 100);
	CS_High();

	// Erase komutunu gönder - non-blocking
	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, command, 4, 100);
	CS_High();

	// Bekleme yok! Background'da erase devam eder
}

static void W25Q_WritePage_NonBlocking(uint16_t page, uint8_t offset, uint32_t size, uint8_t *txBuffer)
{
	uint32_t memAddress = (page * 256) + offset;
	uint8_t data[260];

	data[0] = 0x02;	// Page program
	data[1] = (memAddress >> 16) & 0xFF;
	data[2] = (memAddress >> 8) & 0xFF;
	data[3] = memAddress & 0XFF;

	memcpy(&data[4], txBuffer, 256);

	Write_Enable();
	if((Read_Status_Register_1() & WRITE_ENABLED) != 0)
	{
		CS_Low();
		HAL_SPI_Transmit(W25Q_SPI, data, (size + 4), SPI_Timeout);
		CS_High();
	}
	//Write_Disable();
}

static void CS_Low(void)
{
	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
}

static void CS_High(void)
{
	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}

static void Write_Enable(void)
{
	uint8_t command = 0x06;	// Write enable

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, &command, 1, SPI_Timeout);
	CS_High();
	// HAL_Delay(5) kaldırıldı - performans için
}

static void Write_Disable(void)
{
	uint8_t command = 0x04;	// Write disable

	CS_Low();
	HAL_SPI_Transmit(W25Q_SPI, &command, 1, SPI_Timeout);
	CS_High();
	// HAL_Delay(5) kaldırıldı - performans için
}

static uint16_t Bytes_to_Write(uint32_t remaining_bytes_, uint8_t offset_)
{
	if(remaining_bytes_ + offset_ > 256)
	{
		return (256 - offset_);
	}
	else
	{
		return remaining_bytes_;
	}
}

static void Add_to_Buffer(uint8_t *data_)
{
	if(current_buffer == 1)
	{
		uint16_t startIndex = w25q_counter * 64;
		memcpy(&w25q_txBuffer_1[startIndex], data_, 64);
		/*
		w25q_txBuffer_1[startIndex + 62] = 0x00;
		w25q_txBuffer_1[startIndex + 63] = 0x00;
		*/
	}
	else if(current_buffer == 2)
	{
		uint16_t startIndex = w25q_counter * 64;
		memcpy(&w25q_txBuffer_2[startIndex], data_, 64);
		/*
		w25q_txBuffer_2[startIndex + 62] = 0xFF;
		w25q_txBuffer_2[startIndex + 63] = 0xFF;
		*/
	}
}
