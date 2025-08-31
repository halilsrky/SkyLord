/*
 * filters.h
 *
 *  Created on: Aug 28, 2025
 *      Author: yahya
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_
#include "main.h"

typedef struct {
    float h;      // durum: irtifa (m)
    float v;      // durum: hız (m/s) (pozitif yukarı)
    float abias;  // ivme bias (m/s^2)
    float alpha;  // alpha (irtifa düzeltme)
    float beta;   // beta (hız düzeltme)
    float last_time;
    float dt;
} BaroAccelFilter;

void baf_init(BaroAccelFilter *f, float a0, float alpha, float beta, float dt);
void baf_step(BaroAccelFilter *f, float h_meas, float a_meas, float time);


#endif /* INC_FILTERS_H_ */
