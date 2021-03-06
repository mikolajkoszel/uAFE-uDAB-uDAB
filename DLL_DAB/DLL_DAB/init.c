#include "stdafx.h"





void init_contactors(struct contactors* cont) {
	cont->pc_hv = 1; //stycznik pomocniczny po stronie hv
	cont->pc_lv = 1; //stycznik pomocniczny po stronie lv
	cont->hv = 0; //stycznik główny po stronie hv
	cont->lv = 0; //stycznik główny po stronie lv
	cont->d_v_i = 1000; //pochodna napięcie hv
	cont->rdy_i = 0; //flaga naładowania kondensatorów wejściowych
	cont->v_I_old = -1;
	cont->d_U_o = 1000;
	cont->rdy_o = 0; //flaga naładowania kondensatorów wyjściowych
	cont->U_o_old = -1;
	cont->fi = 0.0; //kąt wysterowania sps
	cont->ts = Conv.Ts;
}
void init_Kalman(struct kalman* measure) {
	measure->Q = 1;
	measure->R = 0.001;
	measure->P_temp = 1;
	measure->x_temp = 0;
}
void init_conv() {
	Conv.started = 0;
	Conv.state = 0;
	Conv.dab_mode = 0;
	Conv.load_type = Nd;
	Conv.enable = 0;
}
