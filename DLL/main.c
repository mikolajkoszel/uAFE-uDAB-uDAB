// Tomasz �wi�chowicz swiechowicz.tomasz@gmail.com

// main.c : Defines the exported functions for the DLL application.

#include "stdafx.h"
#include "stddef.h"

//! Debug :
//. Change the view to expert view using Tools > Settings > Expert settings.
//2. Attach the DLL to the PLECS process using Debug > Attach to process > PLECS.exe.
//3. In main.c, create a breakpoint at the line y = kp * e + ki * i.

DLLEXPORT void plecsSetSizes(struct SimulationSizes* aSizes)
{
	aSizes->numInputs = 12;
	aSizes->numOutputs = 23;
	aSizes->numStates = 0;
	aSizes->numParameters = 6; //number of user parameters passed in
}

void CIC1_filter_init(struct CIC1_struct *CIC, float max_value, Uint16 OSR, Uint16 decimation_ratio)
{
    CIC->decimation_ratio = decimation_ratio;
    CIC->OSR = OSR;
    CIC->div_OSR = 1.0f / (float)OSR;
    CIC->range_modifier = (float)(1UL << 31) / (float)OSR / max_value;
    CIC->div_range_modifier = 1.0f / CIC->range_modifier;
}

void CIC2_filter_init(struct CIC2_struct *CIC, float max_value, Uint16 OSR, Uint16 decimation_ratio)
{
    CIC->decimation_ratio = decimation_ratio;
    CIC->OSR = OSR;
    CIC->div_OSR = 1.0f / (float)OSR;
    CIC->range_modifier = (float)(1UL << 31) / (float)OSR / (float)OSR / max_value;
    CIC->div_range_modifier = 1.0f / CIC->range_modifier;
}

void CIC1_adaptive_filter_init(struct CIC1_adaptive_struct *CIC, float max_value, float OSR)
{
    CIC->range_modifier = (float)(1UL << 31) / OSR / max_value;
    CIC->div_range_modifier = 1.0f / CIC->range_modifier;
}

//This function is automatically called at the beginning of the simulation
DLLEXPORT void plecsStart(struct SimulationState* aState)
{
    memset(&Meas, 0, sizeof(Meas));
    memset(&Conv, 0, sizeof(Conv));
    memset(&PLL, 0, sizeof(PLL));
    memset(&Grid, 0, sizeof(Grid));

	Conv.Ts = aState->parameters[0];
	Conv.L_conv = aState->parameters[1];
    Conv.C_conv = aState->parameters[4];
	Conv.C_dc = aState->parameters[3];
	Conv.R_inr_dc = 3.0f * aState->parameters[5];
    Conv.I_lim_nominal = 40.0f;
    Conv.U_dc_ref = 760.0f;

    Meas_alarm_H.Temperature = 85.0f;
    Meas_alarm_H.U_dc = 680.0f;
    Meas_alarm_L.U_dc = -5.0f;

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    PLL.Ts = Conv.Ts;
    PLL.PI.Kp = 92.0f;
    PLL.PI.Ts_Ti = PLL.Ts / 0.087f;
    PLL.PI.lim_H = 400.0f;
    PLL.PI.lim_L = -400.0f;

    float decimation_PLL = 8.0f;
    float OSR_PLL = (Uint16)(0.02f / (Conv.Ts * decimation_PLL) + 0.5f);
    CIC2_filter_init(&PLL.CIC_w, 400.0f, OSR_PLL, decimation_PLL);

    PLL.f_filter = 50.0f;
    PLL.w_filter = PLL.f_filter * MATH_2PI;
    PLL.state = PLL_omega_init;
    PLL.state_last = PLL_active;

    ///////////////////////////////////////////////////////////////////

    Grid.U_grid_delay = 0.0f;

    Grid.Resonant_I_conv[0].trigonometric.ptr =
    Grid.Resonant_I_conv[1].trigonometric.ptr =
    Grid.Resonant_I_conv[2].trigonometric.ptr =
    Grid.Resonant_U_grid[0].trigonometric.ptr =
    Grid.Resonant_U_grid[1].trigonometric.ptr =
    Grid.Resonant_U_grid[2].trigonometric.ptr = &sincos_table[0];

    Grid.Resonant_I_conv[0].trigonometric_comp.ptr =
    Grid.Resonant_I_conv[1].trigonometric_comp.ptr =
    Grid.Resonant_I_conv[2].trigonometric_comp.ptr =
    Grid.Resonant_U_grid[0].trigonometric_comp.ptr =
    Grid.Resonant_U_grid[1].trigonometric_comp.ptr =
    Grid.Resonant_U_grid[2].trigonometric_comp.ptr = &Grid.U_grid_rot;
    
    Grid.Resonant_I_conv[0].gain =
    Grid.Resonant_I_conv[1].gain =
    Grid.Resonant_I_conv[2].gain =
    Grid.Resonant_U_grid[0].gain =
    Grid.Resonant_U_grid[1].gain =
    Grid.Resonant_U_grid[2].gain = 2.0f * (1.0f / (0.02f - MATH_1_MINUS_1_E * 0.02f)) / (MATH_2PI * 50.0f);

    float OSR = (Uint16)(0.02f / Conv.Ts + 0.5f);

    CIC1_adaptive_filter_init(&Grid.CIC1_U_grid_1h[0], 1000.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_U_grid_1h[1], 1000.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_U_grid_1h[2], 1000.0f, OSR);

    static const float additional_range = 1.5f;
    CIC1_adaptive_filter_init(&Grid.CIC1_P_conv_1h[0], additional_range * 230.0f * 92.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_P_conv_1h[1], additional_range * 230.0f * 92.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_P_conv_1h[2], additional_range * 230.0f * 92.0f, OSR);

    CIC1_adaptive_filter_init(&Grid.CIC1_Q_conv_1h[0], additional_range * 230.0f * 92.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_Q_conv_1h[1], additional_range * 230.0f * 92.0f, OSR);
    CIC1_adaptive_filter_init(&Grid.CIC1_Q_conv_1h[2], additional_range * 230.0f * 92.0f, OSR);

    CIC1_adaptive_global.Ts = Conv.Ts;

    ///////////////////////////////////////////////////////////////////

    CIC1_adaptive_filter_init(&Conv.CIC1_U_dc, 1000.0f, OSR);
    Conv.U_dc_Filter1.Ts_Ti = Conv.Ts / 0.005f;
    Conv.Iq_ref_Filter1.Ts_Ti = Conv.Ts / 0.005f; //0.01f
    Conv.Id_ref_Filter1.Ts_Ti = Conv.Ts / 0.005f;//0.01f

    ///////////////////////////////////////////////////////////////////

    register float alfa2 = 2.0f;
    register float STC2 = Conv.Ts * 1.5f;
    register float pi_i = Conv.L_conv / (alfa2 * STC2);
    register float Ti_i = alfa2 * alfa2 * STC2;

    Conv.PI_I_z.Ts_Ti =
    Conv.PI_I_q.Ts_Ti =
    Conv.PI_I_d.Ts_Ti = Conv.Ts / Ti_i;
    Conv.PI_I_z.Kp =
    Conv.PI_I_q.Kp =
    Conv.PI_I_d.Kp = pi_i;

    ///////////////////////////////////////////////////////////////////

    register float p_pr_i = Conv.L_conv / (15.0f * Conv.Ts); //15.0f
    Conv.Kp_I = p_pr_i;
    register float r_pr_i = Conv.L_conv * MATH_PI / Conv.Ts;
    r_pr_i /= MATH_2PI * 50.0f;

    Conv.resonant_odd_number = 11.0f;
    Conv.resonant_even_number = 2.0f;

    Uint16 i;
    for (i = 0; i < sizeof(Conv.Resonant_I_a_odd) / sizeof(Conv.Resonant_I_a_odd[0]); i++)
    {
       Conv.Resonant_I_a_odd[i].gain =
       Conv.Resonant_I_b_odd[i].gain =
       Conv.Resonant_I_c_odd[i].gain = 2.0f * r_pr_i;// / (float)(2 * i + 1); //2.0f

       Conv.Resonant_I_a_odd[i].trigonometric.ptr =
       Conv.Resonant_I_b_odd[i].trigonometric.ptr =
       Conv.Resonant_I_c_odd[i].trigonometric.ptr = &sincos_table[2 * i];

       Conv.Resonant_I_a_odd[i].trigonometric_comp.ptr =
       Conv.Resonant_I_b_odd[i].trigonometric_comp.ptr =
       Conv.Resonant_I_c_odd[i].trigonometric_comp.ptr = &sincos_table_comp[2 * i];
    }

    for (i = 0; i < sizeof(Conv.Resonant_I_a_even) / sizeof(Conv.Resonant_I_a_even[0]); i++)
    {
       Conv.Resonant_I_a_even[i].gain =
       Conv.Resonant_I_b_even[i].gain =
       Conv.Resonant_I_c_even[i].gain = 2.0f * r_pr_i;// / (float)(2 * i + 2);

       Conv.Resonant_I_a_even[i].trigonometric.ptr =
       Conv.Resonant_I_b_even[i].trigonometric.ptr =
       Conv.Resonant_I_c_even[i].trigonometric.ptr = &sincos_table[2 * i + 1];

       Conv.Resonant_I_a_even[i].trigonometric_comp.ptr =
       Conv.Resonant_I_b_even[i].trigonometric_comp.ptr =
       Conv.Resonant_I_c_even[i].trigonometric_comp.ptr = &sincos_table_comp[2 * i + 1];
    }

    ///////////////////////////////////////////////////////////////////

    for (i = 0; i < SINCOS_HARMONICS; i++)
    {
        sincos_table[i].cosine = cosf(PLL.w_filter * PLL.Ts * (float)(i + 1));
        sincos_table[i].sine = sinf(PLL.w_filter * PLL.Ts * (float)(i + 1));
        float compensation = 2.0f;
        sincos_table_comp[i].cosine = cosf(compensation * PLL.w_filter * PLL.Ts * (float)(i + 1));
        sincos_table_comp[i].sine = sinf(compensation * PLL.w_filter * PLL.Ts * (float)(i + 1));
    }

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    CIC1_adaptive_global.change_timer = -0.2f;
    CIC1_adaptive_global_CLAasm(&CIC1_adaptive_global, 50.0f);
    CIC1_adaptive_global.change_timer = 0.2f;
    CIC1_adaptive_global_CLAasm(&CIC1_adaptive_global, 50.0f);
    CIC1_adaptive_global.change_timer = -0.2f;
    CIC1_adaptive_global_CLAasm(&CIC1_adaptive_global, 50.0f);

	aState_global = aState;
}

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{
    static struct abc_struct I_conv[2] = { 0 };
    static struct abc_struct I_conv_avg = { 0 };

    static float first = 0;
	if (TRIG_0 && ++first >= 100)
	{
		Meas.U_grid.a = aState_global->inputs[2];
		Meas.U_grid.b = aState_global->inputs[3];
		Meas.U_grid.c = aState_global->inputs[4];
        I_conv[0].a = aState_global->inputs[5];
        I_conv[0].b = aState_global->inputs[6];
        I_conv[0].c = aState_global->inputs[7];
        I_conv_avg.a = (I_conv[0].a + I_conv[1].a) * 0.5f;
        I_conv_avg.b = (I_conv[0].b + I_conv[1].b) * 0.5f;
        I_conv_avg.c = (I_conv[0].c + I_conv[1].c) * 0.5f;
        Meas.I_conv.a = I_conv[0].a;
        Meas.I_conv.b = I_conv[0].b;
        Meas.I_conv.c = I_conv[0].c;
		Meas.U_dc = aState_global->inputs[8];
        Conv.P_ref_temp = aState_global->inputs[9];
        Conv.Q_ref_temp = aState_global->inputs[10];
        Conv.change_S_ref = aState_global->inputs[11];

        aState_global->outputs[0] = Conv.duty[0];
        aState_global->outputs[1] = Conv.duty[1];
        aState_global->outputs[2] = Conv.duty[2];

        PLL_calc();

        Grid_analyzer_calc();
        Conv.enable = PLL.RDY;
		Converter_calc();

        aState_global->outputs[12] = Grid.P_conv_1h.a;
        aState_global->outputs[13] = Grid.P_conv_1h.b;
        aState_global->outputs[14] = Grid.P_conv_1h.c;

        aState_global->outputs[15] = Grid.Q_conv_1h.a;
        aState_global->outputs[16] = Grid.Q_conv_1h.b;
        aState_global->outputs[17] = Grid.Q_conv_1h.c;

        aState_global->outputs[18] = Conv.U_dc_filter;

        aState_global->outputs[19] = Grid.sum.P_conv_1h;
        aState_global->outputs[20] = Grid.sum.Q_conv_1h;
        aState_global->outputs[21] = Conv.Id_ref_Filter1.out;
        aState_global->outputs[22] = Conv.Iq_ref_Filter1.out;

    }

	if (TRIG_180)
	{
        I_conv[1].a = aState_global->inputs[5];
        I_conv[1].b = aState_global->inputs[6];
        I_conv[1].c = aState_global->inputs[7];
	}
}
