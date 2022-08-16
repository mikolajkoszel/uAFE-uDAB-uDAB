// Mikolaj Koszel
#include "stdafx.h"
adc_t adc1;
adc_t adc2;
adc_t adc3;
adc_t adc4;
int cnt;
float Ts_Ti = 0.1;
float I_HV_cal, U_HV_cal, U_LV_cal, I_LV_cal;
float ADC_Ts_Ti = 0.001;
void Converter_calc()
{
//	CIC1_adaptive_filter(&CIC1_adaptive_global, &Conv.CIC1_U_dc, Meas.U_HV);
	I_HV_cal = Filter1_calc(Meas.I_HV, &adc1, Ts_Ti);
	U_HV_cal = Filter1_MK(Meas.U_HV, &Conv.U_dc_Filter);
	U_LV_cal = Filter1_calc(Meas.U_LV, &adc3, Ts_Ti);
	I_LV_cal = Filter1_calc(Meas.I_LV, &adc4, Ts_Ti);

if (Conv.run == 0)
	{
		aState_global->outputs[PWM_EN] = CLEAR;
		aState_global->outputs[s_rel_no_o] = CLEAR;
		aState_global->outputs[s_rel_no_dc] = CLEAR;
		aState_global->outputs[s_rel_main_o] = CLEAR;
		aState_global->outputs[s_rel_main_dc] = CLEAR;
		aState_global->outputs[s_rel_dc_o] = SET;
		aState_global->outputs[s_rel_dc_dc] = SET;
		Conv.state = CONV_softstart;
		Conv.state_last = CONV_active;
	}
	else	if (Conv.run == 1) 
			{
				switch (Conv.state)
				{
					case CONV_softstart: //do³adowanie kondenstorów
					{	//precharge(&cont, &d_U_o);
						static float counter_ss;
						static float counter_ss_last;

						static float U_dc_diff;
						static float U_dc_prev;

						static float U_o_diff;
						static float U_o_prev;

						if (Conv.state_last != Conv.state)
						{
							counter_ss = 0.0;
							counter_ss_last = 0.0;
							//Conv.enable = 1.0f;B
							aState_global->outputs[s_rel_no_o] = SET;
							aState_global->outputs[s_rel_no_dc] = SET;
							aState_global->outputs[s_rel_dc_o] = CLEAR;
							aState_global->outputs[s_rel_dc_dc] = CLEAR;
							Conv.state_last = Conv.state;
							U_o_prev = Meas.U_LV;
							U_dc_prev = Meas.U_HV;
						}
						if (counter_ss - counter_ss_last > 0.0125f)
						{
							U_o_diff = Conv.R_inr_dc * Conv.C_dc * (U_LV_cal - U_o_prev) * (1.0f / 0.0125f);
							U_dc_diff = Conv.R_inr_dc * Conv.C_dc * (U_HV_cal - U_dc_prev) * (1.0f / 0.0125f);

							U_o_prev = U_LV_cal;
							U_dc_prev = U_HV_cal;

							Conv.S1 = U_o_diff;
							Conv.S2 = U_dc_diff;
						
							if ((fabsf(U_o_diff - 0.5f) <= 0.5f) && (fabsf(U_dc_diff - 0.5f) <= 0.5f)) //&&
								//(Meas.U_HV > 20.0f) &&
								//(Meas.U_LV > 20.0f))
							{
								aState_global->outputs[s_rel_no_o] = CLEAR;
								aState_global->outputs[s_rel_no_dc] = CLEAR;
								aState_global->outputs[s_rel_main_o]  = SET;
								aState_global->outputs[s_rel_main_dc] = SET;
								Conv.state++;
							}
							counter_ss_last = counter_ss;
						}
						counter_ss += Conv.Ts;
						break;
					}
					case CONV_init_controllers: //okreœlenie typu obci¹¿enia R vs Ÿród³o napiêcia
					{
						if (Conv.dab_mode == battery_slave) {
							PI_U.Kp = 0.5;		//2 Kp of voltage controller (master)
							PI_U.Ki = 10;//0.6;		//Kp of voltage controller (master)
							PI_U.lim_H = 62.0;
							PI_U.lim_L = -62.0;
							PI_U.Ts_Ti = Conv.Ts;

							PI_I.Kp = 0.1;		//0.1 Kp of current controller (master)
							PI_I.Ki = 180.0;	//180 Ki of current controller (master)
							PI_I.lim_H = 90.0;
							PI_I.lim_L = -90.0;
							PI_I.Ts_Ti = Conv.Ts;
							PI_I.e_old = 0.0f;
							PI_I.u_old = 0.0f;
							PI_I.Kerr = 0.2634f;
							PI_I.Kerr_old = 0.2366f;
						}
						if (Conv.dab_mode == battery_master_cascaded_ff) {
							PI_U.Kerr = 0.1901 * 100;		//2 Kp of voltage controller (master)
							PI_U.Kerr_old = 0.1899 * 100;//0.6;		//Kp of voltage controller (master)
							PI_U.lim_H = 62.0;
							PI_U.lim_L = -62.0;
							PI_U.Ts_Ti = Conv.Ts;
							PI_U.e_old = 0.0f;
							PI_U.u_old = 0.0f;

							PI_I.lim_H = 90.0;
							PI_I.lim_L = -90.0;
							PI_I.Ts_Ti = Conv.Ts;
							PI_I.e_old = 0.0f;
							PI_I.u_old = 0.0f;
							PI_I.Kerr = 0.2634f;
							PI_I.Kerr_old = 0.2366f;
							PI_I.Kp = 0.1;		//0.1 Kp of current controller (master)
							PI_I.Ki = 18.0;	//180 Ki of current controller (master)

							PI_I_slave.Kp = 0.1;		//Kp of current controller (master)
							PI_I_slave.Ki = 180.0;	//Ki of current controller (master)
							PI_I_slave.lim_H = 90.0;
							PI_I_slave.lim_L = -90.0;
							PI_I_slave.Ts_Ti = Conv.Ts;
						}
						if (Conv.dab_mode == battery_master_voltage) {
							PI_U.Kp = 0.1;//1
							PI_U.Ki = 5;//1000
							PI_U.Kerr = 1.012;		//2 Kp of voltage controller (master)
							PI_U.Kerr_old = -0.9875;//0.6;		//Kp of voltage controller (master)
							PI_U.lim_H = 17.0f;
							PI_U.lim_L = -17.0f;
							PI_U.Ts_Ti = Conv.Ts;
							PI_U.e_old = 0.0f;
							PI_U.u_old = 0.0f;
					
						}
						if (Conv.dab_mode == battery_master_cur_vol)
						{
							PI_U.Kp = 0.5;//1
							PI_U.Ki = 0.6;
							PI_U.lim_H = 25.0f;
							PI_U.lim_L = -25.0f;
							PI_I.Kp = 5;//1
							PI_I.Ki = 2500;
							PI_I.lim_H = 90.0f;
							PI_I.lim_L = -90.0f;

							PI_I.Ts_Ti = Conv.Ts;
							PI_U.Ts_Ti = Conv.Ts;
						}
						Conv.state++;
						break;
					}
					case CONV_active:	 //praca w pêtli zamkniêtej
					{   Conv.modulation = SPS;
					
						if (Conv.dab_mode == battery_slave) { //0
							aState_global->outputs[PWM_EN] = SET;
							Conv.I_err = (aState_global->inputs[0] - I_LV_cal);
							PI_antiwindup(&PI_I, Conv.I_err);
							//PI_MK(&PI_I, Conv.I_err);
						}
						else if (Conv.dab_mode == battery_master_cascaded_ff) { //1
							aState_global->outputs[PWM_EN] = SET;

							Meas.U_HV = U_HV_cal;
							Meas.I_LV = I_LV_cal;

							Conv.U_err = -(Conv.U_dc_ref - U_HV_cal);
							PI_MK(&PI_U, Conv.U_err);

							Conv.I_err = (PI_U.out - I_LV_cal);

							Conv.I_err = Conv.I_err + 1.5 * (Conv.U_err);

							//if ((Conv.U_err > 5.0f) ||
							/*if (Conv.U_err > 5.0f)
							{
								Conv.I_err = Conv.I_err - 5*(Conv.U_err - 5);
							}
							if (Conv.U_err < -5.0f)
							{
								Conv.I_err = Conv.I_err + 5*(Conv.U_err + 5);
							}*/
								//bagno
							PI_MK(&PI_I, Conv.I_err);
							
							aState_global->outputs[0] = Meas.I_LV;

						}
						else if (Conv.dab_mode == battery_master_voltage) { //2
							
							aState_global->outputs[PWM_EN] = SET;
							Conv.U_err = -(Conv.U_dc_ref - U_HV_cal);
							//PI_MK(&PI_U, Conv.U_err);
							PI_antiwindup(&PI_U, Conv.U_err);
							PI_I.out = PI_U.out;
						}
						else if (Conv.dab_mode == battery_master_cur_vol) //3
						{

							Conv.U_err = -(Conv.U_dc_ref - U_HV_cal);
							PI_antiwindup(&PI_U, Conv.U_err);
							aState_global->outputs[PWM_EN] = SET;
							Conv.I_err = (PI_U.out - I_LV_cal);
							//Conv.I_err = (Conv.U_dc_ref - I_LV_cal);
							PI_antiwindup(&PI_I, Conv.I_err);
						}
						}
						if (Conv.modulation == SPS)
							{
								Conv.fis.ps1 = 0;
								Conv.fis.ps2 = 180;
								Conv.fis.ps3 = PI_I.out;
								if (PI_I.out >= 0) Conv.fis.ps4 = PI_I.out - 180;
								else Conv.fis.ps4 = PI_I.out + 180;

							}
						else if (Conv.modulation == EPS)
							{
								Conv.fis.voltage_ratio_sqrt = sqrt(U_LV_cal / U_HV_cal * 1 / Conv.n);
								Conv.fis.voltage_ratio = U_LV_cal / U_HV_cal * 1 / Conv.n;
								Conv.fis.ps1 = 0; // HV1 k¹t odniesienia
								Conv.fis.ps2 = (Conv.fis.voltage_ratio_sqrt)*180; // HV2 zale¿y tylko od stosunku napiêæ, ale zawiera siê w przedziale 90-180 deg
								if (Conv.fis.ps2 > 180.0f) Conv.fis.ps2 = 180.0f;
								if (Conv.fis.ps2 < 90.0f) Conv.fis.ps2 = 90.0f;

								Conv.fis.ps3 = PI_I.out; //LV1 wyjœcie z regulatora pr¹du wyjœciowego, ograniczenia zale¿¹ od stosunku napiêæ - > przesuwanie siê k¹ta mocy szczytowej w zale¿noœci of ps2
								PI_I.lim_H = 90 - (1 - Conv.fis.voltage_ratio_sqrt) * 90; 
								PI_I.lim_L = -90.0 - (1 - Conv.fis.voltage_ratio_sqrt) * 90;
								
								Conv.fis.ps4 = Conv.fis.ps3 + 180-(Conv.fis.voltage_ratio_sqrt - 1) * 180; // LV2 analogicznie do HV2 
								if ((Conv.fis.ps4 - Conv.fis.ps3) > 180) Conv.fis.ps4 = Conv.fis.ps3 + 180.0;
								if (Conv.fis.ps4 > 180) Conv.fis.ps4 = Conv.fis.ps4 - 360; // przesuniêcie wynikaj¹ce z ograniczenia modulatora (-180,180)
								if (Conv.fis.ps4 < -180) Conv.fis.ps4 = Conv.fis.ps4 + 360;

							}
						aState_global->outputs[0] = U_HV_cal;//Conv.U_dc_Filter.out;
						aState_global->outputs[8] = Conv.I_o_Filter.out;
						aState_global->outputs[9] = PI_I.lim_H;
						aState_global->outputs[10] = PI_I.lim_L;
						aState_global->outputs[11] = Conv.fis.ps1;
						aState_global->outputs[12] = Conv.fis.ps2;
						aState_global->outputs[13] = Conv.fis.ps3;
						aState_global->outputs[14] = Conv.fis.ps4;
						aState_global->outputs[15] = Conv.U_err;
						aState_global->outputs[16] = PI_U.out;
						aState_global->outputs[17] = Conv.I_err;
						aState_global->outputs[18] = PI_I.out;
						
						break;
					}
				}

			}
	
	


