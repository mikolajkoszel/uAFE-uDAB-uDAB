#pragma once

#include "targetver.h"
#include "DllHeader.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>

typedef UINT16 Uint16;
typedef UINT32 Uint32;
typedef UINT64 Uint64;
typedef INT64 int64;
typedef INT32 int32;
typedef INT16 int16;

#define GPIO_SET(pin) aState_global->outputs[pin] = 1
#define GPIO_CLEAR(pin) aState_global->outputs[pin] = 0  
#define GPIO_TOGGLE(pin) aState_global->outputs[pin] ^= 1  
#define GPIO_READ(pin) ((Uint32)aState_global->inputs[pin]) 
#define GPIO_WRITE(pin, val) aState_global->outputs[pin] = val

#define SET 1
#define CLEAR 0

#define PWM_EN 1
#define s_rel_no_o 2
#define s_rel_no_dc 3
#define s_rel_main_o 4
#define s_rel_main_dc 5
#define s_rel_dc_o 6
#define s_rel_dc_dc 7


#include "Controllers.h"
#include "Converter.h"

struct Dab_measurements_struct
{
	float I_HV;
	float U_HV;
	float U_LV;
	float I_LV;
};

struct contactors {
	float v_I_old;
	float d_v_i;
	float rdy_i;
	float U_o_old;
	float d_U_o;
	float rdy_o;
	int pc_hv;
	int pc_lv;
	int hv;
	int lv;
	float fi;
	float ts;
};

struct deriv {
	float measure_old;
	float ts;
	float measure_d;
};


extern struct Dab_measurements_struct Meas;
extern struct contactors cont;
extern struct deriv d_;
extern struct CIC1_adaptive_global_struct CIC1_adaptive_global;
extern struct SimulationState* aState_global;