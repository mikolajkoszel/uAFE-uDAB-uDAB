// Tomasz Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

#define myMATH_E 2.7182818284590452353602874713527
#define myMATH_PI 3.1415926535897932384626433832795
#define myMATH_SQRT3 1.7320508075688772935274463415059
#define myMATH_SQRT2 1.4142135623730950488016887242097

const float MATH_PI = myMATH_PI;
const float MATH_2PI = myMATH_PI * 2.0;
const float MATH_2PI_3 = myMATH_PI * 2.0 / 3.0;
const float MATH_PI_3 = myMATH_PI / 3.0;
const float MATH_1_2PI = 1.0 / (myMATH_PI*2.0);
const float MATH_1_PI = 1.0 / myMATH_PI;
const float MATH_1_3 = 1.0 / 3.0;
const float MATH_1_SQRT3 = 1.0 / myMATH_SQRT3;
const float MATH_1_SQRT2 = 1.0 / myMATH_SQRT2;
const float MATH_SQRT2_3 = myMATH_SQRT2 / 3.0;
const float MATH_SQRT3_2 = myMATH_SQRT3 / 2.0;
const float MATH_SQRT2 = myMATH_SQRT2;
const float MATH_SQRT3 = myMATH_SQRT3;
const float MATH_2_3 = 2.0 / 3.0;
const float MATH_1_325 = 1.0/325.0;
const float MATH_N2_325 = -2.0 / 325.0;
const float MATH_E = myMATH_E;
const float MATH_1_E = 1.0 / myMATH_E;
const float MATH_1_MINUS_1_E = 1.0 - 1.0 / myMATH_E;

void CIC2_filter(struct CIC2_struct *CIC, float input)
{
    if (CIC->decimation_counter-- <= 0.0f) CIC->decimation_counter = CIC->decimation_ratio - 1.0f;
    else return;
    CIC->integrator[1] += CIC->integrator[0];
    CIC->integrator[0] += (int32)(input * CIC->range_modifier);
    register float counter_temp = CIC->counter;
    if (counter_temp-- <= 0.0f) counter_temp = CIC->OSR - 1.0f;
    CIC->counter = counter_temp;
    register int32 div_memory_new = (int32)(counter_temp*CIC_upsample1*CIC->div_OSR);
    if (CIC->div_memory != div_memory_new)
    {
        CIC->div_memory = div_memory_new;
        register int32 *subtractor = (int32 *)&CIC->subtractor + div_memory_new;
        register int32 *decimator = (int32 *)&CIC->decimator_memory + div_memory_new;
        CIC->out = (float)(CIC->integrator[1] - *decimator - *decimator + *subtractor) * CIC->div_OSR * CIC->div_OSR * CIC->div_range_modifier;
        *subtractor = *decimator;
        *decimator = CIC->integrator[1];
    }
}

void CIC1_filter(struct CIC1_struct *CIC, float input)
{
    if(CIC->decimation_counter-- <= 0.0f) CIC->decimation_counter = CIC->decimation_ratio - 1.0f;
    else return;
    CIC->integrator += (int32)(input * CIC->range_modifier);
    register float counter_temp = CIC->counter;
    if (counter_temp-- <= 0.0f) counter_temp = CIC->OSR - 1.0f;
    CIC->counter = counter_temp;
    register int32 div_memory_new = (int32)(counter_temp*CIC_upsample1*CIC->div_OSR);
    if (CIC->div_memory != div_memory_new)
    {
        CIC->div_memory = div_memory_new;
        register int32 *decimator = (int32 *)&CIC->decimator_memory + div_memory_new;
        CIC->out = (float)(CIC->integrator - *decimator) * CIC->div_OSR * CIC->div_range_modifier;
        *decimator = CIC->integrator;
    }
}

void PR_calc(struct PR_struct* PR, float error)
{
    PR->y0 += PR->x1 * PR->Ts;
    PR->x0 += PR->w * PR->w * PR->y0 * PR->Ts;
    PR->x1 = error * PR->Ki - PR->x0;
    PR->out = PR->y0 + error * PR->Kp;
}

void Resonant_filter_calc(struct Resonant_struct* Resonant, float input)
{
    register float error = (input - Resonant->x0) * Resonant->gain;
    register float cosine = Resonant->trigonometric.ptr->cosine;
    register float sine = Resonant->trigonometric.ptr->sine;
    register float temp0 = cosine * Resonant->x0 - sine * Resonant->x1 + sine * error;
    register float temp1 = sine * Resonant->x0 + cosine * Resonant->x1 + cosine * error - error;
    Resonant->x0 = temp0;
    Resonant->x1 = temp1;
    cosine = Resonant->trigonometric_comp.ptr->cosine;
    sine = Resonant->trigonometric_comp.ptr->sine;
    Resonant->y0 = cosine * Resonant->x0 - sine * Resonant->x1;
    Resonant->y1 = sine * Resonant->x0 + cosine * Resonant->x1;
}

float Resonant_calc(struct Resonant_struct* Resonant, float error)
{
    register float cosine = Resonant->trigonometric.ptr->cosine;
    register float sine = Resonant->trigonometric.ptr->sine;
    error *= Resonant->gain;
    register float temp0 = cosine * Resonant->x0 - sine * Resonant->x1 + sine * error;
    register float temp1 = sine * Resonant->x0 + cosine * Resonant->x1 + cosine * error - error;
    Resonant->x0 = temp0;
    Resonant->x1 = temp1;
    Resonant->y0 = Resonant->trigonometric_comp.ptr->cosine * temp0 - Resonant->trigonometric_comp.ptr->sine * temp1;
    return Resonant->y0;
}

float Resonant_mult_calc(struct Resonant_struct* Resonant, float error, Uint16 harmonics)
{
    float sum = 0;

    while (harmonics--)
    {
        register float cosine = Resonant->trigonometric.ptr->cosine;
        register float sine = Resonant->trigonometric.ptr->sine;
        float error_temp = error * Resonant->gain;
        register float temp0 = cosine * Resonant->x0 - sine * Resonant->x1 + sine * error_temp;
        register float temp1 = sine * Resonant->x0 + cosine * Resonant->x1 + cosine * error_temp - error_temp;
        Resonant->x0 = temp0;
        Resonant->x1 = temp1;
        Resonant->y0 = Resonant->trigonometric_comp.ptr->cosine * temp0 - Resonant->trigonometric_comp.ptr->sine * temp1;
        sum += Resonant->y0;
        Resonant++;
    }

    return sum;
}

float Resonant_mult_calc2(struct Resonant_struct *Resonant, Uint16 harmonics)
{
    float sum = 0;

    while (harmonics--)
    {
        register float cosine = Resonant->trigonometric.ptr->cosine;
        register float sine = Resonant->trigonometric.ptr->sine;
        float error_temp = Resonant->input * Resonant->gain;
        register float temp0 = cosine * Resonant->x0 - sine * Resonant->x1 + sine * error_temp;
        register float temp1 = sine * Resonant->x0 + cosine * Resonant->x1 + cosine * error_temp - error_temp;
        Resonant->x0 = temp0;
        Resonant->x1 = temp1;
        Resonant->y0 = Resonant->trigonometric_comp.ptr->cosine * temp0 - Resonant->trigonometric_comp.ptr->sine * temp1;
        sum += Resonant->y0;
        Resonant++;
    }

    return sum;
}

void PR_calc_imp(struct PR_struct* PR, float error)
{
    float b1 = cosf(PR->w * PR->Ts);
    PR->y0 = PR->Ts * (error * PR->Ki - PR->x1 * b1) + 2.0f * b1 * PR->y1 - PR->y2;
    PR->x1 = error * PR->Ki;
    PR->y2 = PR->y1;
    PR->y1 = PR->y0;
    PR->out = PR->y0 + error * PR->Kp;
}

void Filter1_calc(struct Filter1_struct *Filter1, float input)
{
    register float integrator_last = Filter1->out;
    register float y = Filter1->Ts_Ti * (input - integrator_last) - Filter1->Kahan;
    Filter1->out = integrator_last + y;
    Filter1->Kahan = (Filter1->out - integrator_last) - y;
}

void PI_antiwindup_fast(struct PI_struct *PI, float error)
{
    PI->proportional = PI->Kp * error;
    PI->integrator += PI->proportional * PI->Ts_Ti;
    PI->out = PI->integrator + PI->proportional;
    if (PI->out > PI->lim_H)
    {
        PI->out = PI->lim_H;
        PI->integrator = PI->lim_H - PI->proportional;
    }
    if (PI->out < PI->lim_L)
    {
        PI->out = PI->lim_L;
        PI->integrator = PI->lim_L - PI->proportional;
    }
}

void PI_antiwindup_fast_ff(struct PI_struct *PI, float error, float feedforward)
{
    PI->proportional = PI->Kp * error;
    PI->integrator += PI->proportional * PI->Ts_Ti;
    register float feedforward_proportional = PI->proportional + feedforward;
    PI->out = PI->integrator + feedforward_proportional;
    if (PI->out > PI->lim_H)
    {
        PI->out = PI->lim_H;
        PI->integrator = PI->lim_H - feedforward_proportional;
    }
    if (PI->out < PI->lim_L)
    {
        PI->out = PI->lim_L;
        PI->integrator = PI->lim_L - feedforward_proportional;
    }
}

void PI_antiwindup(struct PI_struct *PI, float error)
{
    float integrator_last = PI->integrator;
    PI->proportional = PI->Kp * error;
    PI->integrator += PI->proportional * PI->Ts_Ti;
    PI->out = PI->integrator + PI->proportional;
    if (PI->out > PI->lim_H)
    {
        PI->out = PI->lim_H;
        //safety needed for variable limits
        if ((PI->integrator < PI->lim_H) || (PI->integrator > integrator_last))
        {
            PI->integrator = integrator_last;
        }
    }
    if (PI->out < PI->lim_L)
    {
        PI->out = PI->lim_L;
        //safety needed for variable limits
        if ((PI->integrator > PI->lim_L) || (PI->integrator < integrator_last))
        {
            PI->integrator = integrator_last;
        }
    }
}

void CIC1_adaptive_global_calc(struct CIC1_adaptive_global_struct *CIC_global, float frequency)
{
    float new_osr = 1.0f / (frequency * CIC_global->Ts);
    if (fabs(new_osr - CIC_global->OSR_adaptive[0]) > 0.75f && CIC_global->change_timer < 0.0f)
    {
        new_osr = (float)(Uint32)(new_osr + 0.5f);
        CIC_global->OSR_adaptive[0] = new_osr;
        CIC_global->div_OSR_adaptive[0] = 1.0f / new_osr;
        CIC_global->change_timer = 0.5625f;
    }

    if (CIC_global->change_timer < 0.28125f)
    {
        CIC_global->select_output = 0;
        CIC_global->OSR_adaptive[1] = CIC_global->OSR_adaptive[0];
        CIC_global->div_OSR_adaptive[1] = CIC_global->div_OSR_adaptive[0];
    }
    else
    {
        CIC_global->select_output = 1;
    }

    CIC_global->change_timer -= CIC_global->Ts;

    CIC_global->counter[0] = fminf((CIC_global->counter[0] <= 0.0f) ? CIC_global->OSR_adaptive[0] : CIC_global->counter[0], CIC_global->OSR_adaptive[0]) - 1.0f;
    CIC_global->counter[1] = fminf((CIC_global->counter[1] <= 0.0f) ? CIC_global->OSR_adaptive[1] : CIC_global->counter[1], CIC_global->OSR_adaptive[1]) - 1.0f;
    Uint32 div_memory_new1 = (Uint32)(CIC_global->counter[0] * (float)CIC_upsample2 * CIC_global->div_OSR_adaptive[0]);
    Uint32 div_memory_new2 = (Uint32)(CIC_global->counter[1] * (float)CIC_upsample2 * CIC_global->div_OSR_adaptive[1]);
    div_memory_new1 += div_memory_new1;
    div_memory_new2 += div_memory_new2;

    if (CIC_global->div_memory[0] != div_memory_new1) CIC_global->cycle_enable[0] = 1.0f;
    else CIC_global->cycle_enable[0] = 0.0f;
    if (CIC_global->div_memory[1] != div_memory_new2) CIC_global->cycle_enable[1] = 1.0f;
    else CIC_global->cycle_enable[1] = 0.0f;

    CIC_global->div_memory[0] = div_memory_new1;
    CIC_global->div_memory[1] = div_memory_new2;
}

float CIC1_adaptive_filter(struct CIC1_adaptive_global_struct *CIC_global, struct CIC1_adaptive_struct *CIC, float input)
{
    CIC->integrator += (int32)(input * CIC->range_modifier);

    if (CIC_global->cycle_enable[0])
    {
        register int32 *decimator = &(CIC->decimator_memory[0][CIC_global->div_memory[0] >> 1]);
        CIC->out_temp[0] = (float)(CIC->integrator - *decimator) * CIC_global->div_OSR_adaptive[0] * CIC->div_range_modifier;
        *decimator = CIC->integrator;
    }
    if (CIC_global->cycle_enable[1])
    {
        register int32 *decimator = &(CIC->decimator_memory[1][CIC_global->div_memory[0] >> 1]);
        CIC->out_temp[1] = (float)(CIC->integrator - *decimator) * CIC_global->div_OSR_adaptive[1] * CIC->div_range_modifier;
        *decimator = CIC->integrator;
    }
    return CIC->out = CIC_global->select_output ? CIC->out_temp[1] : CIC->out_temp[0];
}
