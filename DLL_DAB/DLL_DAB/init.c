#include "stdafx.h"





void init_contactors(struct contactors* cont) {
	cont->pc_hv = 1; //stycznik pomocniczny po stronie hv
	cont->pc_lv = 1; //stycznik pomocniczny po stronie lv
	cont->hv = 0; //stycznik g³ówny po stronie hv
	cont->lv = 0; //stycznik g³ówny po stronie lv
	cont->d_v_i = 1000; //pochodna napiêcie hv
	cont->rdy_i = 0; //flaga na³adowania kondensatorów wejœciowych
	cont->v_I_old = -1;
	cont->d_U_o = 1000;
	cont->rdy_o = 0; //flaga na³adowania kondensatorów wyjœciowych
	cont->U_o_old = -1;
	cont->fi = 0.0; //k¹t wysterowania sps
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
