// Tomasz Œwiêchowicz swiechowicz.tomasz@gmail.com

#include "stdafx.h"

struct PLL_struct PLL;
struct Converter_struct Conv;
struct Grid_analyzer_struct Grid;

struct Measurements_struct Meas;
union ALARM alarm = { 0 };

struct Measurements_alarm_struct Meas_alarm_H;
struct Measurements_alarm_struct Meas_alarm_L;

struct trigonometric_struct sincos_table[SINCOS_HARMONICS];
struct trigonometric_struct sincos_table_comp[SINCOS_HARMONICS];
struct CIC1_adaptive_global_struct CIC1_adaptive_global;

struct SimulationState *aState_global;
