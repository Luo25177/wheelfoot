#include "robotparam.h"

void inputInit(Input* input) {
	input->theta		= 0;
	input->thetadot = 0;
	input->x				= 0;
	input->v				= 0;
	input->pitch		= 0;
	input->pitchdot = 0;
}

void outputInit(Output* output) {
	output->Tp		 = 0;
	output->Twheel = 0;
}

float Kcoeff[12][4] = {-174.8494259	,	267.4300679	,	-215.8163833	,	-30.6684885	,
-47.08951496	,	149.1112622	,	-163.4080949	,	89.7361189	,
-16.6500977	,	16.76783828	,	-37.4593255	,	-6.052991868	,
-18.1020216	,	38.34203972	,	-32.40929419	,	19.40841155	,
-52.91637142	,	79.93126275	,	-43.36182962	,	-13.34957244	,
-91.15648414	,	165.8811654	,	-119.8261274	,	40.90343719	,
-31.74259824	,	47.71400163	,	-31.40588853	,	-15.38256628	,
-112.8022143	,	192.1746132	,	-128.6866487	,	41.66642647	,
-144.1310567	,	262.2811518	,	-189.4617429	,	64.67401283	,
334.6725184	,	-505.5296931	,	274.2442902	,	84.43010942	,
-15.74829488	,	26.4720922	,	-17.61437222	,	5.959417414	,
31.29086206	,	-46.70926739	,	25.10624067	,	2.530494731};

