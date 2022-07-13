#include "stdafx.h"
#pragma once

struct Dab_converter_struct
{
	float Ts;
	float R_inr_dc;
	float R_o;
	float C_dc;
	float I_err;
	float U_err;
	float I_o_ref;
	float U_dc_ref;
	float n; //transformer ratio
	float S1;
	float S2;
	float phase_shift;

	int started;
	int enable;
	int run;

	enum converter_state_enum state;
	enum converter_state_enum state_last;
	enum dab_mode_enum dab_mode;
	enum load_type_enum load_type;
	enum modulation_enum modulation;
	 
	struct PI_struct PI_I_o, PI_U_dc;
	struct CIC1_adaptive_struct CIC1_U_dc;
	struct Filter1_struct U_dc_Filter1;
	struct modulation_struct fis;
	struct Filter_struct I_o_Filter;
	struct Filter_struct U_dc_Filter;
};

enum load_type_enum { Nd, Resistor, Voltage_source };
enum load_mode_enum { battery_master_cascaded_ff, battery_master_voltage, battery_slave,  pv };
enum converter_state_enum {CONV_idle, CONV_softstart, CONV_load_type_detection, CONV_active   };
enum modulation_enum {SPS, EPS, DSP};

void Converter_calc();

extern struct Dab_converter_struct Conv;
extern struct PI_struct PI_U;
extern struct PI_struct PI_I;
extern struct PI_struct PI_I_slave;


