// Tomasz Święchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

void Converter_calc()
{
    if(PLL.RDY) CIC1_adaptive_global_CLAasm(&CIC1_adaptive_global, PLL.f_filter);
    else CIC1_adaptive_global_CLAasm(&CIC1_adaptive_global, 50.0f);
    CIC1_adaptive_filter_CLAasm(&CIC1_adaptive_global, &Conv.CIC1_U_dc, Meas.U_dc);
    Filter1_calc_CLAasm(&Conv.U_dc_Filter1, Meas.U_dc);
    Conv.U_dc_filter = Conv.U_dc_Filter1.out;// Conv.CIC1_U_dc.out;

    if (Conv.change_S_ref)
    {
        Conv.P_ref = Conv.P_ref_temp;
        Conv.Q_ref = Conv.Q_ref_temp;
    }
    Conv.Id_ref = Conv.P_ref * Grid.sum.U_grid_1h_div;
    Conv.Iq_ref = Conv.Q_ref * Grid.sum.U_grid_1h_div;
    float Id_ref = Saturation(Conv.Id_ref, -Conv.I_lim, Conv.I_lim);
    float lim_Iq_ref = sqrtf(Conv.I_lim * Conv.I_lim - Id_ref * Id_ref);
    Filter1_calc_CLAasm(&Conv.Id_ref_Filter1, Id_ref);
    Filter1_calc_CLAasm(&Conv.Iq_ref_Filter1, Saturation(Conv.Iq_ref, -lim_Iq_ref, lim_Iq_ref));
    
    ////////////////////////////////////////////////////////////////////////////////////////////

    abc_abg(Meas.I_conv, Meas.I_conv);
    abg_dqz(Meas.I_conv, Meas.I_conv, PLL.trig_table[0].sine, PLL.trig_table[0].cosine);

    abc_abg(Meas.U_grid, Meas.U_grid);
    abg_dqz(Meas.U_grid, Meas.U_grid, PLL.trig_table[0].sine, PLL.trig_table[0].cosine);

    ////////////////////////////////////////////////////////////////////////////////////////////

    float U_grid_phph[3];
    U_grid_phph[0] = fabs(Meas.U_grid.a - Meas.U_grid.b);
    U_grid_phph[1] = fabs(Meas.U_grid.b - Meas.U_grid.c);
    U_grid_phph[2] = fabs(Meas.U_grid.c - Meas.U_grid.a);
    float voltage_max_temp = fmaxf(fmaxf(U_grid_phph[0], U_grid_phph[1]), U_grid_phph[2]);

    ////////////////////////////////////////////////////////////////////////////////////////////

    static float U_grid_phph_running_max;
    U_grid_phph_running_max = fmaxf(voltage_max_temp, U_grid_phph_running_max);

    static float counter_max;
    counter_max += Conv.Ts;
    if (counter_max > 0.02f)
    {
        counter_max = 0.0f;

        Conv.U_grid_phph_max = U_grid_phph_running_max;
        U_grid_phph_running_max = 0.0f;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////

    register float Temp_power = 0;
    Temp_power = fmaxf(Meas.Temperature.a, fmaxf(Meas.Temperature.b, Meas.Temperature.c));
    float duty_power = fmaxf(1.0f - fmaxf((Temp_power - (Meas_alarm_H.Temperature - 5.0f)) * 0.2, 0.0f), 0.0f);
    Conv.I_lim = fmaxf(Conv.I_lim_nominal * duty_power, 1.0f);

    ////////////////////////////////////////////////////////////////////////////////////////////

    if (!Conv.enable || (alarm.all[0] | alarm.all[1]))
    {
        GPIO_CLEAR(PWM_EN_CS);
        GPIO_CLEAR(s_rel_no_DC_CS);
        GPIO_CLEAR(s_rel_main_DC_CS);
        GPIO_CLEAR(s_rel_main_A_CS);
        GPIO_CLEAR(s_rel_main_B_CS);
        GPIO_CLEAR(s_rel_main_C_CS);
        GPIO_CLEAR(s_rel_no_A_CS);
        GPIO_CLEAR(s_rel_no_B_CS);
        GPIO_CLEAR(s_rel_no_C_CS);

        Meas_alarm_L.U_dc = -5.0f;
        Conv.RDY =
        Conv.RDY2 = 0.0f;
        Conv.state = CONV_softstart;
        Conv.state_last = CONV_active;
    }
    else
    {
        switch (Conv.state)
        {
        case CONV_softstart:
        {
            static float counter_ss;
            static float counter_ss_last;
            static float integrated_current;
            static float integrated_voltage;
            static float current_running_max;
            static float U_dc_filter_prev;
            static float U_diff;
            if (Conv.state_last != Conv.state)
            {
                current_running_max =
                integrated_current =
                integrated_voltage =
                counter_ss =
                counter_ss_last = 0.0f;
                
                U_dc_filter_prev = Conv.U_dc_filter;

                GPIO_SET(s_rel_no_DC_CS);
                Conv.state_last = Conv.state;
            }
           
            Conv.duty[0] =
            Conv.duty[1] =
            Conv.duty[2] = 0.0f;

            integrated_current += Conv.Ts * (voltage_max_temp - Meas.U_dc - integrated_voltage - 6.0f) / (2.0f * (Conv.L_conv + 100e-6));
            integrated_current = fmaxf(0.0f, integrated_current);
            integrated_voltage += Conv.Ts * integrated_current / Conv.C_dc;
            
            current_running_max = fmaxf(current_running_max, integrated_current);
            if(integrated_current < 1.0f) integrated_voltage = 0.0f;
            if (counter_ss - counter_ss_last > 0.0125f)
            {
                U_diff = Conv.R_inr_dc * Conv.C_dc * (Conv.U_dc_filter - U_dc_filter_prev)*(1.0f/0.0125f);
                U_dc_filter_prev = Conv.U_dc_filter;

                if ((PLL.Umod_pos > 10.0f) &&
                    (current_running_max == 0.0f) && 
                    (fabsf(U_diff) - 0.5f <= 0.5f) &&
                    (counter_ss > 0.05f) &&
                    fabsf(Meas.U_dc - Conv.U_dc_ref) < 10.0f) Conv.state++;

                current_running_max = 0;
                counter_ss_last = counter_ss;
            }

            counter_ss += Conv.Ts;
            if (counter_ss > 60.0f) alarm.bit.CONV_ERR = 1;
            break;
        }
        case CONV_softstart_C:
        {
            static float switcher1;
            static float switcher2;

            if (Conv.state_last != Conv.state)
            {
                GPIO_SET(s_rel_main_DC_CS);

                Meas_alarm_L.U_dc = Conv.U_dc_ref - 20.0f;
                switcher1 = 1.0f;
                switcher2 = 500.0f;
                Conv.state_last = Conv.state;
            }

            Conv.duty[0] =
            Conv.duty[1] =
            Conv.duty[2] = 0.0f;

            GPIO_CLEAR(PWM_EN_CS);
            if (!switcher2) Conv.state++;
            if (!switcher1--)
            {
                switcher1 = 2.0f;
                switcher2--;
//                GPIO_SET(PWM_EN_CS);
            }
            break;
        }
        case CONV_grid_relay:
        {
            static float counter_ss;
            if (Conv.state_last != Conv.state)
            {
                counter_ss = 0;
                
                GPIO_SET(s_rel_no_A_CS);
                GPIO_SET(s_rel_no_B_CS);
                GPIO_SET(s_rel_no_C_CS);

                Conv.state_last = Conv.state;
            }
            counter_ss += Conv.Ts;

            Conv.duty[0] =
            Conv.duty[1] =
            Conv.duty[2] = 0.0f;

            if (counter_ss > 0.02f)
            {
                GPIO_SET(s_rel_main_A_CS);
                GPIO_SET(s_rel_main_B_CS);
                GPIO_SET(s_rel_main_C_CS);
            }

            if (counter_ss > 0.05f)
            {
                Conv.Id_ref_Filter1.out =
                Conv.Iq_ref_Filter1.out = 0.0f;

                register float div_U_dc = 1.0f / Meas.U_dc;
                Conv.duty[0] = (Meas.U_grid.a) * div_U_dc;
                Conv.duty[1] = (Meas.U_grid.b) * div_U_dc;
                Conv.duty[2] = (Meas.U_grid.c) * div_U_dc;

                Conv.state++;
            }

            break;
        }
        case CONV_active:
        {
            const enum
            {
                enum_PI,
                enum_PR_abc,
                enum_PR_abg,
            }regulator = enum_PR_abg;
            static float counter;
            static float counter_off;
            static float counter_disable;
            if (Conv.state_last != Conv.state)
            {

                GPIO_SET(PWM_EN_CS);
                float counter = 0.0f;
                Conv.PI_I_d.integrator = Meas.U_grid.d;
                Conv.PI_I_q.integrator = Meas.U_grid.q;
                Conv.PI_I_z.integrator = Meas.U_grid.z;

                if (regulator == enum_PR_abc)
                {
                    Conv.Resonant_I_a_odd[0].x0 = Grid.Resonant_U_grid[0].y0;
                    Conv.Resonant_I_b_odd[0].x0 = Grid.Resonant_U_grid[1].y0;
                    Conv.Resonant_I_c_odd[0].x0 = Grid.Resonant_U_grid[2].y0;
                    Conv.Resonant_I_a_odd[0].x1 = Grid.Resonant_U_grid[0].y1;
                    Conv.Resonant_I_b_odd[0].x1 = Grid.Resonant_U_grid[1].y1;
                    Conv.Resonant_I_c_odd[0].x1 = Grid.Resonant_U_grid[2].y1;
                }
                else
                {
                    //mozna zrobic filtr rezonansowy na wartosciach alfa beta i pominac przeksztalcenia ponizej
                    struct transformation_struct U_init;
                    U_init.a = Grid.Resonant_U_grid[0].y0;
                    U_init.b = Grid.Resonant_U_grid[1].y0;
                    U_init.c = Grid.Resonant_U_grid[2].y0;
                    abc_abg(U_init, U_init);
                    Conv.Resonant_I_a_odd[0].x0 = U_init.alf;
                    Conv.Resonant_I_b_odd[0].x0 = U_init.bet;
                    Conv.Resonant_I_c_odd[0].x0 = U_init.gam;
                    U_init.a = Grid.Resonant_U_grid[0].y1;
                    U_init.b = Grid.Resonant_U_grid[1].y1;
                    U_init.c = Grid.Resonant_U_grid[2].y1;
                    abc_abg(U_init, U_init);
                    Conv.Resonant_I_a_odd[0].x1 = U_init.alf;
                    Conv.Resonant_I_b_odd[0].x1 = U_init.bet;
                    Conv.Resonant_I_c_odd[0].x1 = U_init.gam;
                }


                Conv.RDY = 1;
                Conv.state_last = Conv.state;
            }


            //////////////////////////////////////////////////////////////////////////////////

            if (fabs(Conv.U_dc_filter - Conv.U_dc_ref) < 1.0f)
                Conv.RDY2 = 1.0f;

            //////////////////////////////////////////////////////////////////////////////////

            register float sign = PLL.sign;
            Conv.I_ref.a = (Conv.Id_ref_Filter1.out * PLL.trig_table[0].cosine + Conv.Iq_ref_Filter1.out * PLL.trig_table[0].sine * sign) * MATH_SQRT2;
            Conv.I_ref.b = (Conv.Id_ref_Filter1.out * PLL.trig_table[1].cosine + Conv.Iq_ref_Filter1.out * PLL.trig_table[1].sine * sign) * MATH_SQRT2;
            Conv.I_ref.c = (Conv.Id_ref_Filter1.out * PLL.trig_table[2].cosine + Conv.Iq_ref_Filter1.out * PLL.trig_table[2].sine * sign) * MATH_SQRT2;
            abc_abg(Conv.I_ref, Conv.I_ref);
            abg_dqz(Conv.I_ref, Conv.I_ref, PLL.trig_table[0].sine, PLL.trig_table[0].cosine);

            Conv.zero_error = 1.0f;
            if (fabsf(Conv.duty[0] - 0.5f) > 0.5f || fabsf(Conv.duty[1] - 0.5f) > 0.5f || fabsf(Conv.duty[2] - 0.5f) > 0.5f)
                Conv.zero_error = 0;

            //////////////////////////////////////////////////////////////////////////////////
            
            switch (regulator)
            {
                case enum_PI:
                {
                    Conv.I_err.d = Conv.I_ref.d - Meas.I_conv.d;
                    Conv.I_err.q = Conv.I_ref.q - Meas.I_conv.q;
                    Conv.I_err.z = Conv.I_ref.z - Meas.I_conv.z;

                    register float U_ref_lim = Conv.U_dc_filter * MATH_1_SQRT3;
                    Conv.PI_I_d.lim_H =
                    Conv.PI_I_q.lim_H =
                    Conv.PI_I_z.lim_H = U_ref_lim;
                    U_ref_lim = -U_ref_lim;
                    Conv.PI_I_d.lim_L =
                    Conv.PI_I_q.lim_L =
                    Conv.PI_I_z.lim_L = U_ref_lim;
                    PI_antiwindup_fast_CLAasm(&Conv.PI_I_d, Conv.I_err.d);
                    PI_antiwindup_fast_CLAasm(&Conv.PI_I_q, Conv.I_err.q);
                    PI_antiwindup_fast_CLAasm(&Conv.PI_I_z, Conv.I_err.z);

                    register float wL = PLL.w_filter * Conv.L_conv;
                    Conv.U_ref.d = Conv.PI_I_d.out - wL * Meas.I_conv.q;
                    Conv.U_ref.q = Conv.PI_I_q.out + wL * Meas.I_conv.d;
                    Conv.U_ref.z = 0.0f;// Conv.PI_I_z.out;
                    dqz_abg(Conv.U_ref, Conv.U_ref, PLL.trig_table[0].sine, PLL.trig_table[0].cosine);
                    abg_abcn(Conv.U_ref, Conv.U_ref);
                }
                break;
                case enum_PR_abc:
                {
                    Conv.I_err.a = Conv.I_ref.a - Meas.I_conv.a;
                    Conv.I_err.b = Conv.I_ref.b - Meas.I_conv.b;
                    Conv.I_err.c = Conv.I_ref.c - Meas.I_conv.c;

                    register float Kp_I = Conv.Kp_I;
                    Conv.U_ref.a = Conv.I_err.a * Kp_I;
                    Conv.U_ref.b = Conv.I_err.b * Kp_I;
                    Conv.U_ref.c = Conv.I_err.c * Kp_I;

                    Conv.U_ref.a += Resonant_mult_calc_CLAasm(Conv.Resonant_I_a_odd, Conv.I_err.a * Conv.zero_error, Conv.resonant_odd_number);
                    Conv.U_ref.b += Resonant_mult_calc_CLAasm(Conv.Resonant_I_b_odd, Conv.I_err.b * Conv.zero_error, Conv.resonant_odd_number);
                    Conv.U_ref.c += Resonant_mult_calc_CLAasm(Conv.Resonant_I_c_odd, Conv.I_err.c * Conv.zero_error, Conv.resonant_odd_number);

                    //Conv.U_ref.a += Resonant_mult_calc_CLAasm(Conv.Resonant_I_a_even, Conv.I_err.a * Conv.zero_error, Conv.resonant_even_number);
                    //Conv.U_ref.b += Resonant_mult_calc_CLAasm(Conv.Resonant_I_b_even, Conv.I_err.b * Conv.zero_error, Conv.resonant_even_number);
                    //Conv.U_ref.c += Resonant_mult_calc_CLAasm(Conv.Resonant_I_c_even, Conv.I_err.c * Conv.zero_error, Conv.resonant_even_number);
                }
                break;
                case enum_PR_abg:
                {
                    Conv.I_err.alf = Conv.I_ref.alf - Meas.I_conv.alf;
                    Conv.I_err.bet = Conv.I_ref.bet - Meas.I_conv.bet;
                    Conv.I_err.gam = Conv.I_ref.gam - Meas.I_conv.gam;

                    register float Kp_I = Conv.Kp_I;
                    Conv.U_ref.alf = Conv.I_err.alf * Kp_I;
                    Conv.U_ref.bet = Conv.I_err.bet * Kp_I;
                    Conv.U_ref.gam = 0.0f;//Conv.I_err.gam * Kp_I;

                    Conv.U_ref.alf += Resonant_mult_calc_CLAasm(Conv.Resonant_I_a_odd, Conv.I_err.alf * Conv.zero_error, Conv.resonant_odd_number);
                    Conv.U_ref.bet += Resonant_mult_calc_CLAasm(Conv.Resonant_I_b_odd, Conv.I_err.bet * Conv.zero_error, Conv.resonant_odd_number);
                    //Conv.U_ref.gam += Resonant_mult_calc_CLAasm(Conv.Resonant_I_c_odd, Conv.I_err.gam * Conv.zero_error, Conv.resonant_odd_number);

                    //Conv.U_ref.a += Resonant_mult_calc_CLAasm(Conv.Resonant_I_a_even, Conv.I_err.a * Conv.zero_error, Conv.resonant_even_number);
                    //Conv.U_ref.b += Resonant_mult_calc_CLAasm(Conv.Resonant_I_b_even, Conv.I_err.b * Conv.zero_error, Conv.resonant_even_number);
                    //Conv.U_ref.c += Resonant_mult_calc_CLAasm(Conv.Resonant_I_c_even, Conv.I_err.c * Conv.zero_error, Conv.resonant_even_number);

                    abg_abcn(Conv.U_ref, Conv.U_ref);
                }
                break;
            }
 

            register float div_U_dc = 1.0f / fmaxf(Meas.U_dc, 1.0f);
            Conv.duty[0] = Conv.U_ref.a * div_U_dc;
            Conv.duty[1] = Conv.U_ref.b * div_U_dc;
            Conv.duty[2] = Conv.U_ref.c * div_U_dc;
            break;
        }
        default:
        {
            break;
        }
        }

        register float min;
        register float max;
        min = fmaxf(Conv.duty[0], fmaxf(Conv.duty[1], Conv.duty[2]));
        max = fminf(Conv.duty[0], fminf(Conv.duty[1], Conv.duty[2]));
        register float correction_reg = (min + max)*(-0.5f) + 0.5f;
        //register float correction_reg = 0.5f;
        Conv.correction = correction_reg;

        Conv.duty[0] = Conv.duty[0] + correction_reg;
        Conv.duty[1] = Conv.duty[1] + correction_reg;
        Conv.duty[2] = Conv.duty[2] + correction_reg;
    }
}
