// Tomasz �wi�chowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

#pragma once
#ifndef PLL_H_
#define PLL_H_

enum PLL_state_enum
{
    PLL_omega_init,
    PLL_check,
    PLL_active,
    __dummybig_PLL = 300000
};

struct PLL_struct {
    struct SOGI_struct SOGI_alf;
    struct SOGI_struct SOGI_bet;
    struct PI_struct PI;

    float Ts;
    float Umod_pos;
    float theta[3];
    float theta_pos;
    struct trigonometric_struct trig_table[3];

    struct CIC2_struct CIC_w;
    float w;
    float w_filter_internal;
    float w_filter;
    float div_w_filter;
    float f_filter;
    float sign;

    float RDY;
    enum PLL_state_enum state, state_last;
};

extern struct PLL_struct PLL;

void PLL_calc();

#endif /* PLL_H_ */
