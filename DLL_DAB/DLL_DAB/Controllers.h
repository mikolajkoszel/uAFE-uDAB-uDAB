
#include "stdafx.h"

#pragma once
#ifndef Controllers_H_
#define Controllers_H_

#define CIC_upsample2 100

typedef struct {
	float out;
	float Kahan;
} adc_t;

extern float CIC1_adaptive_filter(struct CIC1_adaptive_global_struct* CIC_global, struct CIC1_adaptive_struct* CIC, float input);
void kalman_filter(float measure, struct kalman* kalman);
void precharge(struct contactors* contactors, struct deriv* deriv);
void load_type_detection(struct contactors* contactors, struct deriv* d);
void PI_antiwindup(struct PI_struct* PI, float error);
void PI_MK_antiwindup(struct PI_struct* PI, float error);
void PI_antiwindup2(struct PI_struct* PI, float error);
void derivative(float measure, struct deriv* d);
void PI_marek(struct PI_struct* PI, float error);
void PI_MK(struct PI_struct* PI, float error);
float Filter1_calc(float input, adc_t* add, float Ts_Ti);
float Filter1_MK(float input, struct Filter_struct* filter);


extern float ADC_Ts_Ti;

struct modulation_struct{
	float ps1;
	float ps2;
	float ps3;
	float ps4;
	float voltage_ratio;
	float voltage_ratio_sqrt;
};

struct PI_settings {
	float kpi_s;
	float kii_s;
	float kpv_m;
	float kiv_m;
	float kpi_m;
	float kii_m;
	float ts;
};

struct PI_struct{
	float Vp;
	float Vi;
	float A;
	float Ki;
	float Kp;
	float Kerr;
	float Kerr_old;
	float Ts_Ti;
	float integrator;
	float proportional;
	float lim_H;
	float lim_L;
	float out;

	float u_old;
	float e_old;
};

struct kalman 
{
	float x;
	float x_1;
	float P;
	float P_1;
	float K;
	float Q;
	float R;
	float P_temp;
	float x_temp;
};

struct Filter_struct
{
	float out;
	float out_last;
	float in_last;

};

struct Filter1_struct
{
	float Ts_Ti;
	float out;
	float Kahan;
};

struct CIC1_adaptive_struct
{
	int32 integrator;
	int32 decimator_memory[2][CIC_upsample2];
	float out_temp[2];
	float out;
	float range_modifier;
	float div_range_modifier;
};

struct CIC1_adaptive_global_struct
{
	float OSR_adaptive[2];
	float div_OSR_adaptive[2];
	int32 div_memory[2];
	float counter[2];
	float cycle_enable[2];
	float select_output;
	float change_timer;
	float Ts;
};

#endif /* Controllers_H_ */