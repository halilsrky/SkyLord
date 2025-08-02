/*
 * l86_gnss.c
 *
 *  Created on: Jul 11, 2025
 *      Author: bilal
 */

#include "l86_gnss.h"

UART_HandleTypeDef *huart_gnss;
char gnss_rx_buffer[BUFFER_SIZE * 2];
char gps_buffer[BUFFER_SIZE];

static char *gps_GNRMC_start_point = NULL;
static char *gps_GPGGA_start_point = NULL;
static uint8_t is_data_valid;
static char *current_char;
static char current_data[DATA_SIZE];
static uint8_t counter;
static char msg[MSG_SIZE];
static float non_formatted_latitude;
static float non_formatted_longitude;
static float non_formatted_time;
static uint32_t non_formatted_date;

static void set_baud_rate(L86_GNSS_BAUD_RATE baud_rate);
static uint8_t calculate_checksum(const char *data);
static void process_data(char *rx_buffer, uint16_t buffer_size);
static void get_GNRMC_data(gps_data_t *gps_data_);
static void get_GPGGA_data(gps_data_t *gps_data_);
static void format_data(gps_data_t *gps_data_);

void L86_GNSS_Init(UART_HandleTypeDef *huart_gnss_, L86_GNSS_BAUD_RATE baud_rate)
{
	huart_gnss = huart_gnss_;
	set_baud_rate(baud_rate);
	HAL_UART_Receive_DMA(huart_gnss, (uint8_t *)gnss_rx_buffer, BUFFER_SIZE * 2);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == huart_gnss)
	{
		process_data(gnss_rx_buffer, BUFFER_SIZE);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == huart_gnss)
	{
		process_data(&gnss_rx_buffer[BUFFER_SIZE], BUFFER_SIZE);
	}
}

void L86_GNSS_Update(gps_data_t *gps_data_)
{
	get_GNRMC_data(gps_data_);
	get_GPGGA_data(gps_data_);
	format_data(gps_data_);
}

void L86_GNSS_Print_Info(gps_data_t *gps_data_, UART_HandleTypeDef *huart_Seri_Port)
{
	memset(msg, 0, MSG_SIZE);

	if(gps_data_->is_valid == VALID)
	{
		snprintf(msg, MSG_SIZE, "Latitude: %f %c, Longitude: %f %c, Time: %u.%u.%u, Date: %u/%u/%u\r\n"
					"Speed: %f, Course: %f, Satellites in use: %u, HDOP: %f, Altitude: %f, Geoid height: %f, Orthometric height: %f\r\n",
					gps_data_->latitude, gps_data_->N_S, gps_data_->longitude, gps_data_->E_W,
					gps_data_->time[0], gps_data_->time[1], gps_data_->time[2], gps_data_->date[0], gps_data_->date[1], gps_data_->date[2],
					gps_data_->speed_over_ground, gps_data_->course_over_ground, gps_data_->satellites_in_use, gps_data_->HDOP,
					gps_data_->altitude, gps_data_->geoid_height, gps_data_->orthometric_height);
	}
	else
	{
		memcpy(msg, "Invalid Data!\r\n", 15);
	}

	HAL_UART_Transmit(huart_Seri_Port, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

static void set_baud_rate(L86_GNSS_BAUD_RATE baud_rate)
{
	char commend_buffer[COMMEND_BUFFER_SIZE];

	memset(commend_buffer, 0, COMMEND_BUFFER_SIZE);
	snprintf(commend_buffer, COMMEND_BUFFER_SIZE, "PMTK251,%ul", baud_rate);
	uint8_t checksum = calculate_checksum(commend_buffer);

	memset(commend_buffer, 0, COMMEND_BUFFER_SIZE);
	snprintf(commend_buffer, COMMEND_BUFFER_SIZE, "$PMTK251,%ul*%02X\r\n", baud_rate, checksum);

	HAL_UART_Transmit(huart_gnss, (uint8_t *)commend_buffer, strlen(commend_buffer), 100);
	HAL_Delay(100);

	HAL_UART_DeInit(huart_gnss);
	huart_gnss->Instance = GNSS_UART;
	huart_gnss->Init.BaudRate = baud_rate;
	huart_gnss->Init.WordLength = UART_WORDLENGTH_8B;
	huart_gnss->Init.StopBits = UART_STOPBITS_1;
	huart_gnss->Init.Parity = UART_PARITY_NONE;
	huart_gnss->Init.Mode = UART_MODE_TX_RX;
	huart_gnss->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart_gnss->Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(huart_gnss);
}

static uint8_t calculate_checksum(const char *data)
{
    uint8_t check_sum = 0;
    int i = 0;
	while(data[i] != '\0')
	{
        check_sum ^= data[i];
        i++;
	}
	return check_sum;
}

static void process_data(char *rx_buffer, uint16_t buffer_size)
{
	memcpy(gps_buffer, rx_buffer, buffer_size);
}

static void get_GNRMC_data(gps_data_t *gps_data_)
{
	gps_GNRMC_start_point = strstr(gps_buffer, "GNRMC");

	if(gps_GNRMC_start_point != NULL && *(gps_GNRMC_start_point + 17) == VALID)
	{
		is_data_valid = 1;
	}
	else
	{
		is_data_valid = 0;
		gps_data_->is_valid = INVALID;
	}

	if(is_data_valid == 1)
	{
		memset(current_data, 0, DATA_SIZE);
		counter = 0;
		current_char = gps_GNRMC_start_point;
		while(*current_char != '*')
		{
			current_data[counter] = *current_char;
			counter++;
			current_char++;
		}

		sscanf(current_data, "GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%lu,,,%c",
				&gps_data_->non_fixed_time, &gps_data_->is_valid, &gps_data_->non_fixed_latitude, &gps_data_->N_S,
				&gps_data_->non_fixed_longitude, &gps_data_->E_W, &gps_data_->speed_over_ground, &gps_data_->course_over_ground,
				&gps_data_->non_fixed_date ,&gps_data_->positioning_mode);
	}

	gps_GNRMC_start_point = NULL;
}

static void get_GPGGA_data(gps_data_t *gps_data_)
{
	gps_GPGGA_start_point = strstr(gps_buffer, "GPGGA");

	if(gps_GPGGA_start_point != NULL)
	{
		if(gps_data_->is_valid == VALID)
		{
			memset(current_data, 0, DATA_SIZE);
			counter = 0;
			current_char = gps_GPGGA_start_point;
			while(*(current_char) != '*')
			{
				current_data[counter] = *current_char;
				counter++;
				current_char++;
			}

			sscanf(current_data, "GPGGA,%f,%f,%c,%f,%c,%u,%u,%f,%f,M,%f,M,,",
					&gps_data_->non_fixed_time, &gps_data_->non_fixed_latitude, &gps_data_->N_S,
					&gps_data_->non_fixed_longitude, &gps_data_->E_W, &gps_data_->fix_status, &gps_data_->satellites_in_use,
					&gps_data_->HDOP, &gps_data_->altitude, &gps_data_->geoid_height);
		}

		gps_GPGGA_start_point = NULL;
	}
}

static void format_data(gps_data_t *gps_data_)
{
	if(gps_data_->is_valid == VALID)
	{
		// format latitude
		non_formatted_latitude = gps_data_->non_fixed_latitude;
		gps_data_->latitude = (float)floor(non_formatted_latitude / 100);
		non_formatted_latitude -= gps_data_->latitude * 100;
		non_formatted_latitude /= 60;
		gps_data_->latitude += non_formatted_latitude;

		// format langitude
		non_formatted_longitude = gps_data_->non_fixed_longitude;
		gps_data_->longitude = (float)floor(non_formatted_longitude / 100);
		non_formatted_longitude -= gps_data_->longitude * 100;
		non_formatted_longitude /= 60;
		gps_data_->longitude += non_formatted_longitude;

		// format time
		non_formatted_time = gps_data_->non_fixed_time;
		gps_data_->time[0] = (uint8_t)floor(non_formatted_time / 10000);
		non_formatted_time -= gps_data_->time[0] * 10000;
		gps_data_->time[0] += 3;
		if(gps_data_->time[0] >= 24)
		{
			gps_data_->time[0] -= 24;
		}
		gps_data_->time[1] = (uint8_t)floor(non_formatted_time / 100);
		non_formatted_time-= gps_data_->time[1] * 100;
		gps_data_->time[2] = (uint8_t)floor(non_formatted_time);

		// format date
		non_formatted_date = gps_data_->non_fixed_date;
		gps_data_->date[0] = (uint8_t)(non_formatted_date / 10000);
		non_formatted_date -= gps_data_->date[0] * 10000;
		gps_data_->date[1] = (uint8_t)(non_formatted_date / 100);
		non_formatted_date -= gps_data_->date[1] * 100;
		gps_data_->date[2] = (uint8_t)(non_formatted_date);

		// format height
		gps_data_->orthometric_height = gps_data_->altitude - gps_data_->geoid_height;
	}
}
