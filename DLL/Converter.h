// Tomasz Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

#pragma once
#ifndef Compensator_H_
#define Converter_H_

enum Converter_state_enum
{
    CONV_softstart,
    CONV_softstart_C,
    CONV_grid_relay,
    CONV_active,
    __dummybig_CONV = 300000
};

struct Converter_struct
{
    struct CIC1_adaptive_struct CIC1_U_dc;
    float U_dc_filter;
    float U_dc_ref;
    struct Filter1_struct U_dc_Filter1;
    float U_grid_phph_max;

    struct transformation_struct I_ref, I_err, U_ref;

    float resonant_odd_number;
    float resonant_even_number;
    float zero_error;
    float Kp_I;
    struct Resonant_struct Resonant_I_a_odd[25];
    struct Resonant_struct Resonant_I_b_odd[25];
    struct Resonant_struct Resonant_I_c_odd[25];
    struct Resonant_struct Resonant_I_a_even[25];
    struct Resonant_struct Resonant_I_b_even[25];
    struct Resonant_struct Resonant_I_c_even[25];

    float P_ref, Q_ref;
    float Id_ref, Iq_ref;
    float P_ref_temp, Q_ref_temp;
    float change_S_ref;
    struct Filter1_struct Id_ref_Filter1, Iq_ref_Filter1;
    struct PI_struct PI_I_d, PI_I_q, PI_I_z;

    float Ts;
    float I_lim;
    float I_lim_nominal;
    float L_conv;
    float C_conv;
    float C_dc;
    float R_inr_dc;

    float duty[3];
    float correction;
    float enable;
    float RDY, RDY2;
    enum Converter_state_enum state, state_last;
};

extern struct Converter_struct Conv;

void Converter_calc();

#endif /* Compensator_H_ */
