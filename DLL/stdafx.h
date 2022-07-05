// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

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

#define Resonant_mult3_calc_CLAasm Resonant_mult3_calc 
#define Resonant_mult3_calc2_CLAasm Resonant_mult3_calc2 
#define Resonant_filter_calc_CLAasm Resonant_filter_calc 
#define Filter1_calc_CLAasm Filter1_calc 
#define PI_antiwindup_fast_ff_CLAasm PI_antiwindup_fast_ff 
#define PI_antiwindup_fast_CLAasm PI_antiwindup_fast 
#define PI_antiwindup_CLAasm PI_antiwindup 
#define Resonant_mult_calc_CLAasm Resonant_mult_calc 
#define CIC2_filter_CLAasm CIC2_filter 
#define CIC1_filter_CLAasm CIC1_filter 
#define Resonant_calc_CLAasm Resonant_calc 
#define CIC1_adaptive_filter_CLAasm CIC1_adaptive_filter 
#define CIC1_adaptive_global_CLAasm CIC1_adaptive_global_calc 
#define Kalman_calc_CPUasm Kalman_calc 

#define GPIO_SET(pin) aState_global->outputs[pin] = 1  
#define GPIO_CLEAR(pin) aState_global->outputs[pin] = 0  
#define GPIO_TOGGLE(pin) aState_global->outputs[pin] ^= 1  
#define GPIO_READ(pin) ((Uint32)aState_global->inputs[pin]) 
#define GPIO_WRITE(pin, val) aState_global->outputs[pin] = val

#define s_rel_main_A_CS 4
#define s_rel_main_B_CS 5
#define s_rel_main_C_CS 6
#define s_rel_no_A_CS 7
#define s_rel_no_B_CS 8
#define s_rel_no_C_CS 9
#define s_rel_no_DC_CS 10
#define s_rel_main_DC_CS 11

#define PWM_EN_CS 3

#define TRIG_0 aState_global->inputs[0]
#define TRIG_180 aState_global->inputs[1]

typedef union {
    float *ptr; //Aligned to lower 16-bits
    Uint32 pad; //32-bits
}CLA_FPTR;

typedef union {
    struct trigonometric_struct *ptr; //Aligned to lower 16-bits
    Uint32 pad; //32-bits
}CLA_TRIGPTR;

#include "Controllers.h"
#include "PLL.h"
#include "Grid_analyzer.h"
#include "Converter.h"

struct ALARM_BITS {
    Uint32 I_comp_a_H : 1;
    Uint32 I_comp_a_L : 1;
    Uint32 I_comp_b_H : 1;
    Uint32 I_comp_b_L : 1;

    Uint32 I_comp_c_H : 1;
    Uint32 I_comp_c_L : 1;
    Uint32 I_comp_n_H : 1;
    Uint32 I_comp_n_L : 1;

    Uint32 U_dc_H : 1;
    Uint32 U_dc_L : 1;
    Uint32 Temperature_H : 1;
    Uint32 Temperature_L : 1;

    Uint32 PLL_ERR : 1;
    Uint32 CONV_ERR : 1;

    Uint32 rsvd1 : 18;
    Uint32 rsvd2 : 32;
};

union ALARM {
    Uint32 all[2];
    struct ALARM_BITS bit;
};

struct Measurements_struct
{
    struct transformation_struct U_grid;
    struct transformation_struct I_conv;
    float U_dc;
    struct abc_struct Temperature;
};

struct Measurements_gain_offset_struct
{
    float U_dc;
    struct abc_struct I_conv;
    struct abc_struct U_grid;
};

struct Measurements_alarm_struct
{
    float U_dc;
    float I_conv;
    float Temperature;
};

extern union ALARM alarm;
extern struct Measurements_struct Meas;
extern struct Measurements_alarm_struct Meas_alarm_H;
extern struct Measurements_alarm_struct Meas_alarm_L;

extern struct trigonometric_struct sincos_table[SINCOS_HARMONICS];
extern struct trigonometric_struct sincos_table_comp[SINCOS_HARMONICS];

extern struct CIC1_adaptive_global_struct CIC1_adaptive_global;

extern struct SimulationState *aState_global;
// reference additional headers your program requires here
