
/*
 * filters.c
 *
 *  Created on: Aug 28, 2025
 *      Author: yahya
 */

#include "filter.h"
#include <math.h>
#include "main.h"

#define KB	0.001
#define TO_SI	(float)0.00981

float last_time = 0;

void baf_init(BaroAccelFilter *f, float a0, float alpha, float beta, float time)
{

	f->h = 0.0;
    f->v = 0;
    f->abias = a0 * TO_SI;
    f->alpha = alpha;
    f->beta = beta;
    f->last_time = time;
}

// h_meas: baro'dan hesaplanmış anlık irtifa (m)
// a_meas: ivme sensörünün z-ekseni ölçümü (m/s^2), + yukarı (sensör eksenini uygun yönde ayarla)
void baf_step(BaroAccelFilter *f, float h_meas, float a_meas, float time)
{
	float dt = time - f->last_time;
	dt = dt / 1000;
	f->dt = dt;
	f->last_time = time;
	a_meas = a_meas * TO_SI;

    // 1) Öngörü: ivmeyi kullanarak hız/durum tahmini
    float a_corr = a_meas - f->abias;   // bias düzeltilmiş ivme
    float v_pred = f->v + a_corr * dt;
    float h_pred = f->h + v_pred * dt;  // bir adım öngörü (basit Euler)

    // 2) Ölçüm hatası
    float r = h_meas - h_pred;

    // 3) Güncelleme (α–β)
    f->h = h_pred + f->alpha * r;
    // v güncellemesi: beta bölü dt ile r kullan
    f->v = v_pred + (f->beta / dt) * r;

    // 4) Ivme bias adaptif güncelle (very slow)
    // Eğer baro güvenliyse residual'dan bias'ı azalt
    // kb çok küçük olmalı (örn 1e-3 .. 1e-4)
    f->abias += KB * r; // r pozitifse bias azalması/artanına göre ayarla

    // (Opsiyonel) limitleme
    if (f->abias > 10.0f) f->abias = 10.0f;
    if (f->abias < -10.0f) f->abias = -10.0f;
}
