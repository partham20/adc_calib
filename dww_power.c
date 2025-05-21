/*
 * dww_power.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Varun.R
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dww_parameter.h"
#define Branch_Volt 230
#if 0
//dww_branch_parameter *b_ptr;
int i;
unsigned int dwMathBuff;
unsigned int kw_gain;


void dww_kW_Calc() {

    for (i=0; i<18; i++) {

    if ((b_ptr->dww_RMS_Current[i] != 0)) {
        dwMathBuff = (b_ptr->dww_RMS_Current[i] * b_ptr->dww_branch_Volt[i]);
        b_ptr->dww_branch_kW[i] = dwMathBuff / 1000;
    }
    }
}

void dww_kVA_Calc() {

    for (i=0; i<18; i++) {
        dwMathBuff = ((unsigned long)b_ptr->dww_RMS_Current[i] * Branch_Volt);
        b_ptr->dww_branch_kVA[18] = dwMathBuff / 1000;

    }
}

void dww_kVAR_Calc() {

    for (i=0; i<18; i++) {
        b_ptr->dww_branch_kVAR[18] = sqrt((b_ptr->dww_branch_kVA[i] * b_ptr->dww_branch_kVA[i]) - (b_ptr->dww_branch_kW[i] * b_ptr->dww_branch_kW[i]));
    }

}
#endif


