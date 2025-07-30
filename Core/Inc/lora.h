#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include "main.h"
#include "bme280.h"
#include "bmi088.h"

extern UART_HandleTypeDef huart2;

typedef union Key {
    uint16_t key16;
    uint8_t key8[2];
} key;

typedef union Address {
    uint16_t address16;
    uint8_t address8[2];
} address;

typedef struct Lorastruct {
    uint8_t baudRate;
    uint8_t airRate;
    uint8_t packetSize;
    uint8_t power;
    key loraKey;
    address loraAddress;
    uint8_t channel;
    //default values
    uint8_t netId;
    uint8_t ambientNoise;
    uint8_t RSSI;
    uint8_t transmissonMode;
    uint8_t repeater;
    uint8_t LBT;
    uint8_t worMode;
    uint8_t worCycle;
    uint8_t serialParity;
} lorastruct;



// netid
//#define LORA_NETID (uint8_t)(0x00 << 0)
// baud rate
#define LORA_BAUD_1200 (uint8_t)(0x00 << 5)
#define LORA_BAUD_2400 (uint8_t)(0x01 << 5)
#define LORA_BAUD_4800 (uint8_t)(0x02 << 5)
#define LORA_BAUD_9600 (uint8_t)(0x03 << 5)
#define LORA_BAUD_19200 (uint8_t)(0x04 << 5)
#define LORA_BAUD_38400 (uint8_t)(0x05 << 5)
#define LORA_BAUD_57600 (uint8_t)(0x06 << 5)
#define LORA_BAUD_115200 (uint8_t)(0x07 << 5)

// serial parity (default)
#define LORA_PARITY_8N1 				(uint8_t)(0x00 << 3)
#define LORA_PARITY_8O1 				(uint8_t)(0x01 << 3)
#define LORA_PARITY_8E1 				(uint8_t)(0x02 << 3)

// air data rate
#define LORA_AIR_RATE_0_3k 				(uint8_t)(0x00 << 0)
#define LORA_AIR_RATE_1_2k 				(uint8_t)(0x01 << 0)
#define LORA_AIR_RATE_2_4k 				(uint8_t)(0x02 << 0)
#define LORA_AIR_RATE_4_8k 				(uint8_t)(0x03 << 0)
#define LORA_AIR_RATE_9_6k 				(uint8_t)(0x04 << 0)
#define LORA_AIR_RATE_19_2k 			(uint8_t)(0x05 << 0)
#define LORA_AIR_RATE_38_4k 			(uint8_t)(0x06 << 0)
#define LORA_AIR_RATE_62_5k 			(uint8_t)(0x07 << 0)

//Sub packet setting
#define LORA_SUB_PACKET_240_BYTES 		(uint8_t)(0x00 << 6)
#define LORA_SUB_PACKET_128_BYTES 		(uint8_t)(0x01 << 6)
#define LORA_SUB_PACKET_64_BYTES 		(uint8_t)(0x02 << 6)
#define LORA_SUB_PACKET_32_BYTES 		(uint8_t)(0x03 << 6)

// RSSI AMBIENT NOISE (default)
#define LORA_RSSI_AMBIENT_NOISE_DISABLE (uint8_t)(0x00 << 5)
#define LORA_RSSI_AMBIENT_NOISE_ENABLE	(uint8_t)(0x01 << 5)

// Operating status log
#define LORA_STATUS_LOG_DISABLE 		(uint8_t)(0x00 << 2)
#define LORA_STATUS_LOG_ENABLE 			(uint8_t)(0x01 << 2)

// Transmitting Power
#define LORA_POWER_37dbm 				(uint8_t)(0x00 << 0)

// RSSI enable
#define LORA_RSSI_DISABLE 				(uint8_t)(0x00 << 7)
#define LORA_RSSI_ENABLE				(uint8_t)(0x01 << 7)

// Transmisson Mode (default)
#define LORA_TRANSMISSION_TRANSPARENT 	(uint8_t)(0x00 << 6)
#define LORA_TRANSMISSION_FIXED 		(uint8_t)(0x01 << 6)

// Enable repeater (default)
#define LORA_REPEATER_DISABLE 			(uint8_t)(0x00 << 5)
#define LORA_REPEATER_ENABLE  			(uint8_t)(0x01 << 5)

// LBT enable (default)
#define LORA_LBT_DISABLE 				(uint8_t)(0x00 << 4)
#define LORA_LBT_ENABLE 				(uint8_t)(0x01 << 4)

// Wor transceiver control (default)
#define LORA_WOR_RECEIVER 				(uint8_t)(0x00 << 3)
#define LORA_WOR_TRANSMITTER 			(uint8_t)(0x01 << 3)

// Wor cycle (default)
#define LORA_WOR_0_500 					(uint8_t)(0x00 << 0)
#define LORA_WOR_1000 					(uint8_t)(0x01 << 0)
#define LORA_WOR_1500 					(uint8_t)(0x02 << 0)
#define LORA_WOR_2000 					(uint8_t)(0x03 << 0)
#define LORA_WOR_2500 					(uint8_t)(0x04 << 0)
#define LORA_WOR_3000 					(uint8_t)(0x05 << 0)
#define LORA_WOR_3500 					(uint8_t)(0x06 << 0)
#define LORA_WOR_4000 					(uint8_t)(0x07 << 0)

// Lora yapılandırmasını data dizisine yazarü

void lora_configure(lorastruct *config);
void lora_activate();
void lora_deactivate();
#endif

