// Tomasz Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

void Grid_analyzer_calc()
{
    register float wTs;
    wTs = Grid.U_grid_delay * PLL.w_filter * PLL.Ts;
    Grid.U_grid_rot.sine = sinf(wTs);
    Grid.U_grid_rot.cosine = cosf(wTs);

    Resonant_filter_calc_CLAasm(&Grid.Resonant_U_grid[0], Meas.U_grid.a);
    Resonant_filter_calc_CLAasm(&Grid.Resonant_U_grid[1], Meas.U_grid.b);
    Resonant_filter_calc_CLAasm(&Grid.Resonant_U_grid[2], Meas.U_grid.c);

    Resonant_filter_calc_CLAasm(&Grid.Resonant_I_conv[0], Meas.I_conv.a);
    Resonant_filter_calc_CLAasm(&Grid.Resonant_I_conv[1], Meas.I_conv.b);
    Resonant_filter_calc_CLAasm(&Grid.Resonant_I_conv[2], Meas.I_conv.c);

    CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_U_grid_1h[0], sqrtf(0.5f * (Grid.Resonant_U_grid[0].x0 * Grid.Resonant_U_grid[0].x0 + Grid.Resonant_U_grid[0].x1 * Grid.Resonant_U_grid[0].x1)) );
    CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_U_grid_1h[1], sqrtf(0.5f * (Grid.Resonant_U_grid[1].x0 * Grid.Resonant_U_grid[1].x0 + Grid.Resonant_U_grid[1].x1 * Grid.Resonant_U_grid[1].x1)) );
    CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_U_grid_1h[2], sqrtf(0.5f * (Grid.Resonant_U_grid[2].x0 * Grid.Resonant_U_grid[2].x0 + Grid.Resonant_U_grid[2].x1 * Grid.Resonant_U_grid[2].x1)) );
    Grid.U_grid_1h.a = Grid.CIC1_U_grid_1h[0].out;
    Grid.U_grid_1h.b = Grid.CIC1_U_grid_1h[1].out;
    Grid.U_grid_1h.c = Grid.CIC1_U_grid_1h[2].out;
    Grid.sum.U_grid_1h = (Grid.U_grid_1h.a + Grid.U_grid_1h.b + Grid.U_grid_1h.c);

    ///////////////////////////////////////////////////////////////////

    static struct abc_struct Q_conv_temp;
    Q_conv_temp.a = 0.5f * (Grid.Resonant_U_grid[0].x1 * Grid.Resonant_I_conv[0].y0 - Grid.Resonant_U_grid[0].x0 * Grid.Resonant_I_conv[0].y1);
    Q_conv_temp.b = 0.5f * (Grid.Resonant_U_grid[1].x1 * Grid.Resonant_I_conv[1].y0 - Grid.Resonant_U_grid[1].x0 * Grid.Resonant_I_conv[1].y1);
    Q_conv_temp.c = 0.5f * (Grid.Resonant_U_grid[2].x1 * Grid.Resonant_I_conv[2].y0 - Grid.Resonant_U_grid[2].x0 * Grid.Resonant_I_conv[2].y1);
    Grid.Q_conv_1h.a = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_Q_conv_1h[0], Q_conv_temp.a);
    Grid.Q_conv_1h.b = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_Q_conv_1h[1], Q_conv_temp.b);
    Grid.Q_conv_1h.c = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_Q_conv_1h[2], Q_conv_temp.c);
    Grid.sum.Q_conv_1h = Grid.Q_conv_1h.a + Grid.Q_conv_1h.b + Grid.Q_conv_1h.c;

    static struct abc_struct P_conv_temp;
    P_conv_temp.a = 0.5f * (Grid.Resonant_U_grid[0].x0 * Grid.Resonant_I_conv[0].y0 + Grid.Resonant_U_grid[0].x1 * Grid.Resonant_I_conv[0].y1);
    P_conv_temp.b = 0.5f * (Grid.Resonant_U_grid[1].x0 * Grid.Resonant_I_conv[1].y0 + Grid.Resonant_U_grid[1].x1 * Grid.Resonant_I_conv[1].y1);
    P_conv_temp.c = 0.5f * (Grid.Resonant_U_grid[2].x0 * Grid.Resonant_I_conv[2].y0 + Grid.Resonant_U_grid[2].x1 * Grid.Resonant_I_conv[2].y1);
    Grid.P_conv_1h.a = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_P_conv_1h[0], P_conv_temp.a);
    Grid.P_conv_1h.b = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_P_conv_1h[1], P_conv_temp.b);
    Grid.P_conv_1h.c = CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Grid.CIC1_P_conv_1h[2], P_conv_temp.c);
    Grid.sum.P_conv_1h = Grid.P_conv_1h.a + Grid.P_conv_1h.b + Grid.P_conv_1h.c;

    Grid.S_conv_1h.a = sqrtf(Grid.P_conv_1h.a * Grid.P_conv_1h.a + Grid.Q_conv_1h.a * Grid.Q_conv_1h.a);
    Grid.S_conv_1h.b = sqrtf(Grid.P_conv_1h.b * Grid.P_conv_1h.b + Grid.Q_conv_1h.b * Grid.Q_conv_1h.b);
    Grid.S_conv_1h.c = sqrtf(Grid.P_conv_1h.c * Grid.P_conv_1h.c + Grid.Q_conv_1h.c * Grid.Q_conv_1h.c);
    Grid.sum.S_conv_1h = Grid.S_conv_1h.a + Grid.S_conv_1h.b + Grid.S_conv_1h.c;

    Grid.PF_conv_1h.a = Grid.P_conv_1h.a / fmaxf(Grid.S_conv_1h.a, 1.0f);
    Grid.PF_conv_1h.b = Grid.P_conv_1h.b / fmaxf(Grid.S_conv_1h.b, 1.0f);
    Grid.PF_conv_1h.c = Grid.P_conv_1h.c / fmaxf(Grid.S_conv_1h.c, 1.0f);
    Grid.sum.PF_conv_1h = (fabs(Grid.P_conv_1h.a) + fabs(Grid.P_conv_1h.b) + fabs(Grid.P_conv_1h.c)) / fmaxf(Grid.sum.S_conv_1h, 1.0f);

    Grid.U_grid_1h_div.a = 1.0f / Grid.U_grid_1h.a;
    Grid.U_grid_1h_div.b = 1.0f / Grid.U_grid_1h.b;
    Grid.U_grid_1h_div.c = 1.0f / Grid.U_grid_1h.c;
    Grid.sum.U_grid_1h_div = 1.0f / Grid.sum.U_grid_1h;

    Grid.iq_conv.a = Grid.Q_conv_1h.a * Grid.U_grid_1h_div.a;
    Grid.iq_conv.b = Grid.Q_conv_1h.b * Grid.U_grid_1h_div.b;
    Grid.iq_conv.c = Grid.Q_conv_1h.c * Grid.U_grid_1h_div.c;

    Grid.id_conv.a = Grid.P_conv_1h.a * Grid.U_grid_1h_div.a;
    Grid.id_conv.b = Grid.P_conv_1h.b * Grid.U_grid_1h_div.b;
    Grid.id_conv.c = Grid.P_conv_1h.c * Grid.U_grid_1h_div.c;
}
