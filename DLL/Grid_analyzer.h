// Tomasz Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

#pragma once
#ifndef GRID_ANALYZER_H_
#define GRID_ANALYZER_H_

struct Grid_analyzer_struct
{
    float U_grid_delay;
    struct trigonometric_struct U_grid_rot;

    struct Resonant_struct Resonant_U_grid[3];
    struct Resonant_struct Resonant_I_conv[3];

    struct CIC1_adaptive_struct CIC1_U_grid_1h[3];
    struct CIC1_adaptive_struct CIC1_Q_conv_1h[3];
    struct CIC1_adaptive_struct CIC1_P_conv_1h[3];

    struct abc_struct U_grid_1h;
    struct abc_struct U_grid_1h_div;
    struct abc_struct P_conv_1h;
    struct abc_struct Q_conv_1h;
    struct abc_struct S_conv_1h;
    struct abc_struct PF_conv_1h;

    struct abc_struct iq_conv, id_conv;

    struct
    {
        float U_grid_1h;
        float U_grid_1h_div;
        float P_conv_1h;
        float Q_conv_1h;
        float S_conv_1h;
        float PF_conv_1h;
    }sum;
};

extern struct Grid_analyzer_struct Grid;

void Grid_analyzer_calc();

#endif /* GRID_ANALYZER_H_ */
