/*
 * data_logger.h
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */

#ifndef INC_DATA_LOGGER_H_
#define INC_DATA_LOGGER_H_

void data_logger_init();
void log_datas(float altitude, float lat, float lon, float time, float altitude_pressure, float temperature, float humidity);
void log_normal_packet_data(unsigned char* packet_data);
void flush_packet_buffer(void);
void force_flush_buffer(void);

#endif /* INC_DATA_LOGGER_H_ */
