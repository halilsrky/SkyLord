/*
 * w25_flash_memory.h
 *
 *  Created on: Aug 24, 2025
 *      Author: bilal
 */

#ifndef INC_W25_FLASH_MEMORY_H_
#define INC_W25_FLASH_MEMORY_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define CS_Pin W25_FLASH_CS_Pin
#define CS_Port W25_FLASH_CS_GPIO_Port

#define SPI_Timeout 5000

#define WIP_BUSY 0x01			// Erase/Write In Progress (BUSY)
#define WRITE_ENABLED 0x02		// Write Enable Latch (WEL)

void W25Q_Init(SPI_HandleTypeDef *w25q_spi);
void W25Q_Reset(void);
void W25Q_Read(uint16_t startPage, uint8_t offset, uint32_t size, uint8_t *rxBuffer);
void W25Q_FastRead(uint16_t startPage, uint8_t offset, uint32_t size, uint8_t *rxBuffer);
void W25Q_EraseSector(uint16_t sector);
void W25Q_Write(uint16_t page, uint8_t offset, uint32_t size, uint8_t *txBuffer);
void W25Q_WriteToBufferFlushOnSectorFull(uint8_t *data);
uint8_t Read_Status_Register_1(void);

#endif /* INC_W25_FLASH_MEMORY_H_ */
