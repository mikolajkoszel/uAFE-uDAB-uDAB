// Mikolaj Koszel

#include "stdafx.h"

struct Dab_converter_struct Conv;
struct Dab_measurements_struct Meas;


struct CIC1_adaptive_global_struct CIC1_adaptive_global;

struct SimulationState* aState_global;

//struct modulation_struct fis;
struct PI_struct PI_U;
struct PI_struct PI_I;
struct PI_struct PI_I_slave;

float ADC_Ts_Ti;