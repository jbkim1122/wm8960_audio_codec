#ifndef _WM8960_H_ 
#define _WM8960_H_ 

#include "wm8960_register.h"
#include "data_types.h"
#include "bt_types.h"
#include "../app_config.h"

/******************************************************************************
*                         Definitions
******************************************************************************/
#define WM8960_Addr (0x34 >> 1) 
#define SIDETONE_DEFAULT_LEVEL 4

/******************************************************************************
*                         Type Defines
******************************************************************************/

enum {
    AUDIO_OUTPUT_STEREO,
    AUDIO_OUTPUT_MONO,
    AUDIO_OUTPUT_INVALID
};

enum {
    LEFT_AUDIO_BYPASS_GAIN_MINUS_21db   = 0x07,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_18db   = 0x06,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_15db   = 0x05,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_12db   = 0x04,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_9db    = 0x03,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_6db    = 0x02,
    LEFT_AUDIO_BYPASS_GAIN_MINUS_3db    = 0x01,
    LEFT_AUDIO_BYPASS_GAIN_0db          = 0x00,
    LEFT_AUDIO_BYPASS_GAIN_MAX          = 0x08
};

typedef uint8_t sidetone_val_t;

/******************************************************************************
*                       Variables
******************************************************************************/
uint8_t sidetone_level = SIDETONE_DEFAULT_LEVEL;

/******************************************************************************
*                          Function prototype
******************************************************************************/

void WM8960_Aux_control(uint8_t toggle_val);
void WM8960_Mic_MuteControl(uint8_t toggle_val);
void WM8960_Mic_AmpVolumeControl(Input_AMP_MGAIN_T mgain);
void WM8960_Mic_DigitalVolumeControl(I_MIC_DVOL_T mic_volume_val);
void WM8960_Output_DigitalVolumeControl(unsigned char DVOL);
void WM8960_Output_AmpVolumeControl(unsigned char AVOL);
void WM8960_SetOutput_Config(uint8_t val);
void WM8960_set_sample_freq(uint8_t sample_freq);
void WM8960_MIC_BoostGain_ctrl(uint8_t val);
void WM8960_Sidetone_ctrl(uint8_t val);
void WM8960_Sidetone_vol_ctrl(uint8_t val);

void WM8960_Write_Reg(uint8_t regAddr, uint16_t regVal);
uint16_t WM8960_Read_Reg(uint8_t regAddr);
#endif