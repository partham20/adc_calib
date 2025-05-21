

//#include <stdio.h>
//#include <stdint.h>

#ifndef DWW_PARAMETER_H
#define DWW_PARAMETER_H

#define MAX_CT_LENGTH 18

#if 0
typedef struct {
    unsigned int dww_RMS_Current[18];
    unsigned int dww_branch_Volt[18];
    unsigned int dww_max_curr[18];
    unsigned int dww_min_curr[18];
    unsigned int dww_branch_kW[18];
    unsigned int dww_branch_kVA[18];
    unsigned int dww_branch_kVAR[18];
    unsigned int dww_Loadper[18];
    unsigned int dww_CTCurrDemand_1hr[18];
    unsigned int dww_CTCurrDemand_24hr[18];

} dww_branch_parameter;
#endif
typedef struct {
    unsigned int demand_chk_1sec :1;
    unsigned int demand_chk_1hr :1;
    unsigned int demand_chk_24hr :1;

} dww_structflag;



#if 0
typedef struct {
    unsigned int Channel_1;
    unsigned int Channel_2;
    unsigned int Channel_3;
    unsigned int Channel_4;
    unsigned int Channel_5;
    unsigned int Channel_6;
    unsigned int Channel_7;
    unsigned int Channel_8;
    unsigned int Channel_9;
    unsigned int Channel_10;
    unsigned int Channel_11;
    unsigned int Channel_12;
    unsigned int Channel_13;
    unsigned int Channel_14;
    unsigned int Channel_15;
    unsigned int Channel_16;
    unsigned int Channel_17;
    unsigned int Channel_18;

} DWW_BRANCH_PARAMETER;

typedef union {
    unsigned int array[18];
    DWW_BRANCH_PARAMETER  word;

} DWW_BRANCH_UNION;
#endif

typedef struct {
    unsigned int Current;
    unsigned int Voltage_LL;
    unsigned int Voltage_LN;
    unsigned int Loadper;
    unsigned int Frequency;
    unsigned int Mincurrent;
    unsigned int Maxcurrent;
    unsigned int KW;
    unsigned int KVA;
    unsigned int KVAR;
    unsigned int CTCurrDemand_1hr;
    unsigned int CTMaxCurrDemand_1hr;
    unsigned int CTMaxCurrDemand_24hr;

} DWW_BRANCH_PARAMETER;

typedef struct {
    DWW_BRANCH_PARAMETER dww_channel[MAX_CT_LENGTH];
    unsigned int Current_buffer[MAX_CT_LENGTH];
    unsigned int Voltage_buffer[MAX_CT_LENGTH];

} DWW_CHANNEL_PARAMETER;

/*
 * This structure defines the Voltage data
 * obtained from the S-Board through CAN
 */
typedef struct {
    unsigned int wAdcVoltBuff[3];
    unsigned int L2L_Volt_Phase_RY;
    unsigned int L2L_Volt_Phase_YB;
    unsigned int L2L_Volt_Phase_BR;
    unsigned int L2N_Volt_Phase_R;
    unsigned int L2N_Volt_Phase_Y;
    unsigned int L2N_Volt_Phase_B;
    unsigned int Freq_R_Phase;


} DWW_CAN_PARAMETER;

#endif
