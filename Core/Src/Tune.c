/*
 * Tune.c
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */

#include "TuningInterface/TuningInterface.h"
#include "Tune.h"

Tune_Int_t TuneIntList0[NUM_INT_TUNE_LIST0] = {

};

Tune_Float_t TuneFloatList0[NUM_FLOAT_TUNE_LIST0]={
		varF(AP, 0.5, 10.0),
		varF(AI, 0.5, 10.0),
		varF(AD, 0.0, 0.1),
		varF(BP, 0.5, 10.0),
		varF(BI, 0.5, 10.0),
		varF(BD, 0.0, 0.1),
		varF(CP, 0.5, 10.0),
		varF(CI, 0.5, 10.0),
		varF(CD, 0.0, 0.1),
		varF(DP, 0.5, 10.0),
		varF(DI, 0.5, 10.0),
		varF(DD, 0.0, 0.1),

		varF(tol_xy, 0.0, 0.40),
		varF(tol_z, 0.0, 10.0),
		varF(f_tol_xy, 0.0, 0.1),
		varF(f_tol_z, 0.0, 5.0),
		varF(kp[0], 0.0, 5.0),
		varF(ki[0], 0.0, 5.0),
		varF(kd[0], 0.0, 5.0),
		varF(kp[1], 0.0, 5.0),
		varF(ki[1], 0.0, 5.0),
		varF(kd[1], 0.0, 5.0),
};

Tune_Int_t TuneIntList1[NUM_INT_TUNE_LIST1] = {

};

/*
* 		[][0]= minimum speed
* 		[][1]= x-coordinate
* 		[][2]= y-coordinate
* 		[][3]= z-coordinate
* 		[][4]= xy pid output
* 		[][5]= Point Lock
* 		[][6]= Curve Control Radius
*/
Tune_Float_t TuneFloatList1[NUM_FLOAT_TUNE_LIST1]={
		varF(point1[0], 0.0, 10.0),
		varF(point1[1], 0.0, 3.0),
		varF(point1[2], 0.0, 3.0),
		varF(point1[3], 0.0, 180.0),
		varF(point1[4], 0.0, 5.0),
};

Tune_Int_t TuneIntList2[NUM_INT_TUNE_LIST2] = {

};

Tune_Float_t TuneFloatList2[NUM_FLOAT_TUNE_LIST2]={

};

Tune_Int_t TuneIntList3[NUM_INT_TUNE_LIST3] = {

};

Tune_Float_t TuneFloatList3[NUM_FLOAT_TUNE_LIST3]={

};

Tune_Int_t TuneIntList4[NUM_INT_TUNE_LIST4] = {

};

Tune_Float_t TuneFloatList4[NUM_FLOAT_TUNE_LIST4]={

};

