#include"stdafx.h"

void PI_antiwindup(struct PI_struct* PI, float error)
{
    float integrator_last = PI->integrator;
    PI->proportional = PI->Kp * error;
    //PI->integrator += PI->proportional * PI->Ts_Ti;
	PI->integrator += PI->Ki* PI->Ts_Ti*error;
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

void PI_MK(struct PI_struct* PI, float error)
{
	PI->out = PI->u_old + PI->Kerr*error - PI->Kerr_old*PI->e_old;
	if (PI->out > PI->lim_H)		PI->out = PI->lim_H;
	else if (PI->out < PI->lim_L)	PI->out = PI->lim_L;
	PI->e_old = error;
	PI->u_old = PI->out;
}

void PI_marek(struct PI_struct* PI, float error)
{

	PI->out = PI->u_old + PI->proportional * (error - PI->e_old) + 0.5f * (PI->Ts_Ti) * (error + PI->e_old);
	if (PI->out > PI->lim_H)		PI->out = PI->lim_H;
	else if (PI->out < PI->lim_L)	PI->out = PI->lim_L;

	PI->e_old = error;
	PI->u_old = PI->out;
}

void PI_antiwindup2(struct PI_struct* PI, float error)
{
	float integrator_last = PI->integrator;
	PI->proportional = PI->Kp * error;
	//PI->integrator += PI->proportional * PI->Ts_Ti;
	PI->integrator += PI->Ki * PI->Ts_Ti * error;
	PI->out = PI->integrator + PI->proportional;
	if (PI->out > PI->lim_H)
	{
		PI->out = PI->lim_H;
		//safety needed for variable limits
		if ((PI->integrator < PI->lim_H) || (PI->integrator < integrator_last))
		{
			PI->integrator = integrator_last;
		}
	}
	if (PI->out < PI->lim_L)
	{
		PI->out = PI->lim_L;
		//safety needed for variable limits
		if ((PI->integrator > PI->lim_L) || (PI->integrator > integrator_last))
		{
			PI->integrator = integrator_last;
		}
	}
}
void PI_antiwindup_fast(struct PI_struct* PI, float error)
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
void precharge(struct contactors* c, struct deriv* d_U_o)
{
	if (c->d_v_i < 100 && c->d_v_i > -100) {
		c->pc_hv = 0;
		c->hv = 1;
		c->rdy_i = 1;
	}
	c->d_v_i = (Meas.U_HV - c->v_I_old) / c->ts;
	//c->d_v_i = 1;
	c->v_I_old = Meas.U_HV;

	if (c->d_U_o < 100 && c->d_U_o > -100) {
		if (Meas.U_LV < Meas.U_HV * 0.0666) {
			c->pc_lv = 0;
			c->lv = 1;
		}
		else {
			c->pc_lv = 0;
			c->lv = 1;
		}
		c->rdy_o = 1;
	}
	c->d_U_o = (Meas.U_LV - c->U_o_old) / c->ts;
	//c->d_U_o = 1;
	c->U_o_old = Meas.U_LV;
	if (c->rdy_i == 1 && c->rdy_o == 1 && Meas.U_LV >= Meas.U_HV * 0.0666) Conv.state = 2;
	else if (c->rdy_i == 1 && c->rdy_o == 1) {
		Conv.state = 1;
		Conv.enable = 1;
	}
}
extern void derivative(float measure, struct deriv* d)
{
	d->measure_d = (measure - d->measure_old) / d->ts;
	d->measure_old = measure;
}
void load_type_detection(struct contactors* c, struct deriv* d)
{
	if(Conv.enable == 1)
	{ 
		if (Meas.I_LV != 0) {
			if (c->pc_lv == 0) Conv.R_o = Meas.U_LV / Meas.I_LV;
			else
			{
				Conv.R_o = (Meas.U_LV - Conv.R_inr_dc * Meas.I_LV) / Meas.I_LV;
			}
		}

		if (Meas.U_LV > Meas.U_HV * 0.0666 && c->fi > 0)
		{
			c->fi -= 0.001;
		}
		else if (Meas.U_LV > Meas.U_HV * 0.0666 && c->fi < 0.01)
		{
			Conv.enable = 0;
		}
		else if (c->fi < 0.2) {
			c->fi += 0.001;
		}
	}
	if(Conv.enable == 0)
	{
		c->d_U_o = (Meas.U_LV - c->U_o_old) / c->ts;
		c->U_o_old = Meas.U_LV;
		if (c->d_U_o > -100 && c->d_U_o < 100)
		{
			if (Meas.U_LV < 0.5 * Meas.U_HV * 0.0666) Conv.load_type = Resistor;
			else Conv.load_type = Voltage_source;
			Conv.state = 2;
			Conv.enable = 1;
		}

		
	}
}
void kalman_filter(float measure, struct kalman* par)
{
	par->x_1 = par->x_temp;
	par->P_1 = par->P_temp + par->Q;
	par->K = par->P / (par->P + par->R);
	par->x = par->x_1 + par->K * (measure - par->x_1);
	par->P = par->P_1 - par->K * (par->P + par->R);
	par->x_temp = par->x;
	par->P_temp = par->P;
}


float Filter1_calc(float input, adc_t* add, float Ts_Ti) {
	float integrator_last = add->out;
	float y = Ts_Ti * (input - integrator_last) - add->Kahan;
	add->out = integrator_last + y;
	add->Kahan = (add->out - integrator_last) - y;
	return add->out;
}

float Filter1_MK(float in, struct Filter_struct* filter) {
	filter->out = 0.94 * filter->out_last + 0.03 * in + 0.03 * filter->in_last;

	filter->out_last = filter->out;
	filter->in_last = in;
	return filter->out;
}

void CIC1_adaptive_global_calc(struct CIC1_adaptive_global_struct* CIC_global, float frequency)
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

float CIC1_adaptive_filter(struct CIC1_adaptive_global_struct* CIC_global, struct CIC1_adaptive_struct* CIC, float input)
{
	CIC->integrator += (int32)(input * CIC->range_modifier);

	if (CIC_global->cycle_enable[0])
	{
		register int32* decimator = &(CIC->decimator_memory[0][CIC_global->div_memory[0] >> 1]);
		CIC->out_temp[0] = (float)(CIC->integrator - *decimator) * CIC_global->div_OSR_adaptive[0] * CIC->div_range_modifier;
		*decimator = CIC->integrator;
	}
	if (CIC_global->cycle_enable[1])
	{
		register int32* decimator = &(CIC->decimator_memory[1][CIC_global->div_memory[0] >> 1]);
		CIC->out_temp[1] = (float)(CIC->integrator - *decimator) * CIC_global->div_OSR_adaptive[1] * CIC->div_range_modifier;
		*decimator = CIC->integrator;
	}
	return CIC->out = CIC_global->select_output ? CIC->out_temp[1] : CIC->out_temp[0];
}