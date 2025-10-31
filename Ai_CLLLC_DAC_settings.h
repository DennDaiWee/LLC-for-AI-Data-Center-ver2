#ifndef AI_CLLLC_DAC_SETTINGS_H_
#define AI_CLLLC_DAC_SETTINGS_H_

void DAC_targets_switching();
void DAC_init(uint32_t);

/* self-define functions and variable */
/* DACA_scope is to observe the values after sensed, DACB_scope is to observe the values in MCU */
typedef enum {
    TEST_A, Iprim, Vprim, Isec, Vsec, Iprim_tank, Isec_tank, iEfuse, vBatt
} DACA_scope;

typedef enum {
    TEST_B, Volt_ctrl_res, Cur_ctrl_res
} DACB_scope;


#endif /* AI_CLLLC_DAC_SETTINGS_H_ */
