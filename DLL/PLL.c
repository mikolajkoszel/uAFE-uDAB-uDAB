// Tomasz  Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

void PLL_calc()
{
    struct abg_struct U_grid;
    abc_ab(U_grid, Meas.U_grid);

    switch (PLL.state)
    {
    case PLL_omega_init:
    {
        static float time_counter;
        static float integrator;
        static float theta;
        static float theta_last;

        theta_last = theta;
        theta = atan2f(U_grid.bet, U_grid.alf);

        if (PLL.state_last != PLL.state)
        {
            PLL.state_last = PLL.state;

            PLL.w_filter = 50.0f * MATH_2PI;

            theta_last = theta;
            PLL.RDY =
                integrator =
                time_counter = 0;
        }

        register float error = theta - theta_last;
        integrator += error - (float)((int32)(error * MATH_1_PI)) * MATH_2PI;

        time_counter += PLL.Ts;
        if (time_counter >= 0.01f)
        {
            float omega_est = integrator * (1.0f / 0.01f);

            PLL.theta_pos = theta;
            PLL.PI.integrator = omega_est;

            PLL.SOGI_alf.x = U_grid.alf;
            PLL.SOGI_bet.x = U_grid.bet;

            if (omega_est > 0)
            {
                PLL.SOGI_alf.qx = U_grid.bet;
                PLL.SOGI_bet.qx = -U_grid.alf;
            }
            else
            {
                PLL.SOGI_alf.qx = -U_grid.bet;
                PLL.SOGI_bet.qx = U_grid.alf;
            }

            PLL.state++;
        }
        break;
    }
    case PLL_check:
    {
        static float synch_ok_counter;
        static float synch_counter;
        static float counter_avg;

        static float w_filter_max;
        static float w_filter_min;

        if (PLL.state_last != PLL.state)
        {
            PLL.state_last = PLL.state;

            counter_avg =
                synch_counter =
                synch_ok_counter = 0;
        }

        w_filter_max = fmaxf(w_filter_max, PLL.w_filter);
        w_filter_min = fminf(w_filter_min, PLL.w_filter);

        //sprawdzamy co 1 okres w celu eliminacji wplywu oscylacji na wartosc usredniona
        counter_avg += PLL.Ts;
        if (counter_avg >= 0.02f)
        {
            if (fabs(w_filter_min - w_filter_max) <= 0.2f) synch_ok_counter++;
            else synch_ok_counter = 0;

            w_filter_max =
                w_filter_min = PLL.w_filter;

            counter_avg = 0;
            synch_counter++;
        }

        if (synch_counter >= 40) PLL.state = PLL_omega_init;
        if (synch_ok_counter >= 5)
        {
            PLL.state++;
            goto error_check;
        }

        goto no_error_check;
    }
    case PLL_active:
    {
        PLL.RDY = 1;
        PLL.w_filter = PLL.w_filter_internal;
    error_check:
        if ((PLL.f_filter < 48.0f || PLL.f_filter > 52.0f) && (PLL.f_filter < 58.0f || PLL.f_filter > 62.0f))
        {
            PLL.state = PLL_omega_init;
        }
    no_error_check:

        PLL.w = fabs(PLL.PI.out);
        PLL.theta_pos += PLL.Ts * PLL.PI.out;
        PLL.theta_pos -= (float)((int32)(PLL.theta_pos * MATH_1_PI)) * MATH_2PI;

        //Wyliczenie kata
        PLL.theta[0] = PLL.theta_pos;
        PLL.theta[1] = PLL.theta_pos - MATH_2PI_3;
        PLL.theta[2] = PLL.theta_pos + MATH_2PI_3;

        PLL.theta[1] -= (float)((int32)(PLL.theta[1] * MATH_1_PI)) * MATH_2PI;
        PLL.theta[2] -= (float)((int32)(PLL.theta[2] * MATH_1_PI)) * MATH_2PI;

        //Odfiltrowana omega
        CIC2_filter_CLAasm(&PLL.CIC_w, PLL.w);
        PLL.w_filter_internal = fmaxf(PLL.CIC_w.out, 300.0f);

        PLL.trig_table[0].cosine = cosf(PLL.theta[0]);
        PLL.trig_table[0].sine = sinf(PLL.theta[0]);
        PLL.trig_table[1].cosine = cosf(PLL.theta[1]);
        PLL.trig_table[1].sine = sinf(PLL.theta[1]);
        PLL.trig_table[2].cosine = cosf(PLL.theta[2]);
        PLL.trig_table[2].sine = sinf(PLL.theta[2]);

        //SOGI dla alf/bet
        SOGI_calc(PLL.SOGI_alf, U_grid.alf, PLL.w * PLL.Ts);
        SOGI_calc(PLL.SOGI_bet, U_grid.bet, PLL.w * PLL.Ts);

        float U_pos_alf;
        float U_pos_bet;
        //Wyznaczenie skladowej zgodnej sygnalu w alf/bet
        if (PLL.PI.out > 0.0f)
        {
            U_pos_alf = PLL.SOGI_alf.x - PLL.SOGI_bet.qx;
            U_pos_bet = PLL.SOGI_bet.x + PLL.SOGI_alf.qx;
            PLL.sign = 1.0f;
        }
        else
        {
            U_pos_alf = PLL.SOGI_alf.x + PLL.SOGI_bet.qx;
            U_pos_bet = PLL.SOGI_bet.x - PLL.SOGI_alf.qx;
            PLL.sign = -1.0f;
        }

        //Regulator PI
        PLL.Umod_pos = sqrtf(U_pos_alf * U_pos_alf + U_pos_bet * U_pos_bet) * 0.5f;
        register float error_PLL = (PLL.trig_table[0].cosine * U_pos_bet - PLL.trig_table[0].sine * U_pos_alf) / fmaxf(PLL.Umod_pos, 1.0f);
        PI_antiwindup_fast_CLAasm(&PLL.PI, error_PLL);
        break;
    }
    default:
        PLL.state = PLL_omega_init;
        break;
    }

    PLL.f_filter = PLL.w_filter * MATH_1_2PI;
    PLL.div_w_filter = 1.0f / PLL.w_filter;
}
