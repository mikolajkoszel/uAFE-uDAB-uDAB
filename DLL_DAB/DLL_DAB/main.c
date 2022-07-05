//Miko³aj Koszel

#include "stdafx.h"
#include "stddef.h"

//Dual active bridge
//hv(high voltage) -> input(i) ->dc link(dc), lv(low voltage) ->output(o)

//Global variables

float  temp;
struct kalman I_o_filter, v_dc_filter;

//Map input and output parameters to meaningful names/



DLLEXPORT void plecsSetSizes(struct SimulationSizes* aSizes)
{
   aSizes->numInputs = 9;
   aSizes->numOutputs = 19;
   aSizes->numStates = 1;
   aSizes->numParameters = 4; //number of user parameters passed in
}

void CIC1_adaptive_filter_init(struct CIC1_adaptive_struct* CIC, float max_value, float OSR)
{
	CIC->range_modifier = (float)(1UL << 31) / OSR / max_value;
	CIC->div_range_modifier = 1.0f / CIC->range_modifier;
}

//This function is automatically called at the beginning of the simulation
DLLEXPORT void plecsStart(struct SimulationState* aState)
{
	memset(&Meas, 0, sizeof(Meas));
	memset(&Conv, 0, sizeof(Conv));
	
	//Assign controller settings from plecs
	Conv.Ts = aState->parameters[0];
	Conv.R_inr_dc = aState->parameters[1];
	Conv.C_dc = aState->parameters[2];
	Conv.n = aState->parameters[3];
	Conv.enable = 0.0f;
	Conv.state = CONV_softstart;
	Conv.state_last = CONV_active;
	init_conv(&Conv);

	init_Kalman(&I_o_filter);

	init_Kalman(&v_dc_filter);


	float OSR = (Uint16)(0.02f / Conv.Ts + 0.5f);

	aState_global = aState;
	Conv.U_dc_Filter1.Ts_Ti = 0.005;//Conv.Ts / 0.001f;
	CIC1_adaptive_filter_init(&Conv.CIC1_U_dc, 1000.0f, OSR);
	Conv.run = 0;
	Converter_calc();
}	

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{
	Conv.I_o_ref = aState_global->inputs[0];
	Conv.U_dc_ref = aState_global->inputs[1];
	Conv.run = aState_global->inputs[2];
	Conv.dab_mode = aState_global->inputs[3];

	Meas.I_LV = aState_global->inputs[5];
	Meas.U_LV = aState_global->inputs[6];
	Meas.I_HV = aState_global->inputs[7];
	Meas.U_HV = aState_global->inputs[8];

	
	//kalman_filter(Meas.
	// , &I_o_filter);
	//kalman_filter(Meas.v_dc, &v_dc_filter);b
	

	Converter_calc();
	Conv.enable = 1.0f;

	//aState_global->outputs[0] = Conv.phase_shift;
	//aState_global->outputs[1] = Conv.enable;
	//aState_global->outputs[2] = 1;
	//aState_global->outputs[8] = Conv.U_err;
	//aState_global->outputs[10] = Conv.state_last;
	
	aState_global = aState;
}




