#ifndef _WM8960_register_h_
#define _WM8960_register_h_

#include "data_types.h"
#include "bt_types.h"

/* 
 * Register address ***************************************************************
 **/

#define left_input_volume_addr                  0x00
#define right_input_volume_addr                 0x01
#define lout1_volume_addr                       0x02
#define rout1_volume_addr                       0x03
#define clocking1_addr                          0x04
#define adc_dac_control1_addr                   0x05
#define adc_dac_control2_addr                   0x06
#define audio_interface1_addr                   0x07
#define clocking2_addr                          0x08
#define audio_interface2_addr                   0x09
#define left_dac_volume_addr                    0x0A
#define right_dac_volume_addr                   0x0B
#define reset_addr                              0x0F
#define filter_3d_control_addr                  0x10
#define alc1_addr                               0x11
#define alc2_addr                               0x12
#define alc3_addr                               0x13
#define noise_gate_addr                         0x14
#define left_adc_volume_addr                    0x15
#define right_adc_volume_addr                   0x16
#define additional_control1_addr                0x17
#define additional_control2_addr                0x18
#define pwr_mgmt1_addr                          0x19
#define pwr_mgmt2_addr                          0x1A
#define additional_control3_addr                0x1B
#define anti_pop1_addr                          0x1C
#define anti_pop2_addr                          0x1D
#define adcl_signal_path_addr                   0x20
#define adcr_signal_path_addr                   0x21
#define left_out_mix1_addr                      0x22
#define right_out_mix2_addr                     0x25
#define mono_out_mix1_addr                      0x26
#define mono_out_mix2_addr                      0x27
#define lout2_volume_addr                       0x28
#define rout2_volume_addr                       0x29
#define monoout_volume_addr                     0x2A
#define input_boost_mixer1_addr                 0x2B
#define input_boost_mixer2_addr                 0x2C
#define bypass1_addr                            0x2D
#define bypass2_addr                            0x2E
#define pwr_mgmt3_addr                          0x2F
#define additional_control4_addr                0x30
#define class_d_control1_addr                   0x31
#define class_d_control3_addr                   0x33
#define pll_n_addr                              0x34
#define pll_k1_addr                             0x35
#define pll_k2_addr                             0x36
#define pll_k3_addr                             0x37


/* 
 * Register mask *****************************************************************
 **/

//Left input volume
#define LEFT_INPUT_VOLUME_IPVU_OFFSET           8
#define LEFT_INPUT_VOLUME_LINMUTE_OFFSET        7
#define LEFT_INPUT_VOLUME_LIZC_OFFSET           6
#define LEFT_INPUT_VOLUME_LINVOL_OFFSET         0

#define LEFT_INPUT_VOLUME_IPVU_MASK             (0x01 << LEFT_INPUT_VOLUME_IPVU_OFFSET)
#define LEFT_INPUT_VOLUME_LINMUTE_MASK          (0x01 << LEFT_INPUT_VOLUME_LINMUTE_OFFSET)
#define LEFT_INPUT_VOLUME_LIZC_MASK             (0x01 << LEFT_INPUT_VOLUME_LIZC_OFFSET)
#define LEFT_INPUT_VOLUME_LINVOL_MASK           (0x3F << LEFT_INPUT_VOLUME_LINVOL_OFFSET)

//Right input volume
#define RIGHT_INPUT_VOLUME_IPVU_OFFSET          8
#define RIGHT_INPUT_VOLUME_RINMUTE_OFFSET       7
#define RIGHT_INPUT_VOLUME_RIZC_OFFSET          6
#define RIGHT_INPUT_VOLUME_RINVOL_OFFSET        0

#define RIGHT_INPUT_VOLUME_IPVU_MASK            (0x01 << RIGHT_INPUT_VOLUME_IPVU_OFFSET)
#define RIGHT_INPUT_VOLUME_RINMUTE_MASK         (0x01 << RIGHT_INPUT_VOLUME_RINMUTE_OFFSET)
#define RIGHT_INPUT_VOLUME_RIZC_MASK            (0x01 << RIGHT_INPUT_VOLUME_RIZC_OFFSET)
#define RIGHT_INPUT_VOLUME_RINVOL_MASK          (0x3F << RIGHT_INPUT_VOLUME_RINVOL_OFFSET)

//LOUT1 volume
#define LOUT1_VOLUME_OUT1VU_OFFSET              8
#define LOUT1_VOLUME_LO1ZC_OFFSET               7
#define LOUT1_VOLUME_LOUT1VOL_OFFSET            0

#define LOUT1_VOLUME_OUT1VU_MASK                (0x01 << LOUT1_VOLUME_OUT1VU_OFFSET)
#define LOUT1_VOLUME_LO1ZC_MASK                 (0x01 << LOUT1_VOLUME_LO1ZC_OFFSET)
#define LOUT1_VOLUME_LOUT1VOL_MASK              (0x7F << LOUT1_VOLUME_LOUT1VOL_OFFSET)

//ROUT1 volume
#define ROUT1_VOLUME_OUT1VU_OFFSET              8
#define ROUT1_VOLUME_RO1ZC_OFFSET               7
#define ROUT1_VOLUME_ROUT1VOL_OFFSET            0

#define ROUT1_VOLUME_OUT1VU_MASK                (0x01 << ROUT1_VOLUME_OUT1VU_OFFSET)
#define ROUT1_VOLUME_RO1ZC_MASK                 (0x01 << ROUT1_VOLUME_RO1ZC_OFFSET)
#define ROUT1_VOLUME_ROUT1VOL_MASK              (0x7F << ROUT1_VOLUME_ROUT1VOL_OFFSET)

//Clocking 1 ~ 2
#define CLOCKING1_ADCDIV_OFFSET                 6
#define CLOCKING1_DACDIV_OFFSET                 3
#define CLOCKING1_SYSCLKDIV_OFFSET              1
#define CLOCKING1_CLKSEL_OFFSET                 0

#define CLOCKING1_ADCDIV_MASK                   (0x07 << CLOCKING1_ADCDIV_OFFSET)
#define CLOCKING1_DACDIV_MASK                   (0x07 << CLOCKING1_DACDIV_OFFSET)
#define CLOCKING1_SYSCLKDIV_MASK                (0x03 << CLOCKING1_SYSCLKDIV_OFFSET)
#define CLOCKING1_CLKSEL_MASK                   (0x01 << CLOCKING1_CLKSEL_OFFSET)

#define CLOCKING2_DCLKDIV_OFFSET                6
#define CLOCKING2_BCLKDIV_OFFSET                0

#define CLOCKING2_DCLKDIV_MASK                  (0x07 << CLOCKING2_DCLKDIV_OFFSET)
#define CLOCKING2_BCLKDIV_MASK                  (0x0F << CLOCKING2_BCLKDIV_MASK)

//ADC & DAC Control(CTR1)
#define ADC_DAC_CONTROL1_DACDIV2_OFFSET         7
#define ADC_DAC_CONTROL1_ADCPOL_OFFSET          5
#define ADC_DAC_CONTROL1_DACMU_OFFSET           3
#define ADC_DAC_CONTROL1_DEEMPH_OFFSET          1
#define ADC_DAC_CONTROL1_ADCHPD_OFFSET          0

#define ADC_DAC_CONTROL1_DACDIV2_MASK           (0x01 << ADC_DAC_CONTROL1_DACDIV2_OFFSET)
#define ADC_DAC_CONTROL1_ADCPOL_MASK            (0x03 << ADC_DAC_CONTROL1_ADCPOL_OFFSET)
#define ADC_DAC_CONTROL1_DACMU_MASK             (0x01 << ADC_DAC_CONTROL1_DACMU_OFFSET)
#define ADC_DAC_CONTROL1_DEEMPH_MASK            (0x03 << ADC_DAC_CONTROL1_DEEMPH_OFFSET)
#define ADC_DAC_CONTROL1_ADCHPD_MASK            (0x01 << ADC_DAC_CONTROL1_ADCHPD_OFFSET)

//ADC & DAC Control(CTR2)
#define ADC_DAC_CONTROL2_DACPOL_OFFSET          5
#define ADC_DAC_CONTROL2_DACSMM_OFFSET          3
#define ADC_DAC_CONTROL2_DACMR_OFFSET           2
#define ADC_DAC_CONTROL2_DACSLOPE_OFFSET        1

#define ADC_DAC_CONTROL2_DACPOL_MASK            (0x03 << ADC_DAC_CONTROL2_DACPOL_OFFSET)
#define ADC_DAC_CONTROL2_DACSMM_MASK            (0x01 << ADC_DAC_CONTROL2_DACSMM_OFFSET)
#define ADC_DAC_CONTROL2_DACMR_MASK             (0x01 << ADC_DAC_CONTROL2_DACMR_OFFSET)
#define ADC_DAC_CONTROL2_DACSLOPE_MASK          (0x01 << ADC_DAC_CONTROL2_DACSLOPE_OFFSET)

//Audio Interface 1 ~ 2
#define AUDIO_INTERFACE1_ALRSWAP_OFFSET         8
#define AUDIO_INTERFACE1_BCLKINV_OFFSET         7
#define AUDIO_INTERFACE1_MS_OFFSET              6
#define AUDIO_INTERFACE1_DLRSWAP_OFFSET         5
#define AUDIO_INTERFACE1_LRP_OFFSET             4
#define AUDIO_INTERFACE1_WL_OFFSET              2
#define AUDIO_INTERFACE1_FORMAT_OFFSET          0

#define AUDIO_INTERFACE1_ALRSWAP_MASK           (0x01 << AUDIO_INTERFACE1_ALRSWAP_OFFSET)
#define AUDIO_INTERFACE1_BCLKINV_MASK           (0x01 << AUDIO_INTERFACE1_BCLKINV_OFFSET)
#define AUDIO_INTERFACE1_MS_MASK                (0x01 << AUDIO_INTERFACE1_MS_OFFSET)
#define AUDIO_INTERFACE1_DLRSWAP_MASK           (0x01 << AUDIO_INTERFACE1_DLRSWAP_OFFSET)
#define AUDIO_INTERFACE1_LRP_MASK               (0x01 << AUDIO_INTERFACE1_LRP_OFFSET)
#define AUDIO_INTERFACE1_WL_MASK                (0x03 << AUDIO_INTERFACE1_WL_OFFSET)
#define AUDIO_INTERFACE1_FORMAT_MASK            (0x03 << AUDIO_INTERFACE1_FORMAT_OFFSET)

#define AUDIO_INTERFACE2_ALRCGPIO_OFFSET        6
#define AUDIO_INTERFACE2_WL8_OFFSET             5
#define AUDIO_INTERFACE2_DACCOMP_OFFSET         3
#define AUDIO_INTERFACE2_ADCCOMP_OFFSET         1
#define AUDIO_INTERFACE2_LOOPBACK_OFFSET        0

#define AUDIO_INTERFACE2_ALRCGPIO_MASK          (0x01 << AUDIO_INTERFACE2_ALRCGPIO_OFFSET)
#define AUDIO_INTERFACE2_WL8_MASK               (0x01 << AUDIO_INTERFACE2_WL8_OFFSET)
#define AUDIO_INTERFACE2_DACCOMP_MASK           (0x03 << AUDIO_INTERFACE2_DACCOMP_OFFSET)
#define AUDIO_INTERFACE2_ADCCOMP_MASK           (0x03 << AUDIO_INTERFACE2_ADCCOMP_OFFSET)
#define AUDIO_INTERFACE2_LOOPBACK_MASK          (0x01 << AUDIO_INTERFACE2_LOOPBACK_OFFSET)

//Left DAC volume
#define LEFT_DAC_VOLUME_DACVU_OFFSET            8
#define LEFT_DAC_VOLUME_LDACVOL_OFFSET          0


#define LEFT_DAC_VOLUME_DACVU_MASK              (0x01 << LEFT_DAC_VOLUME_DACVU_OFFSET)
#define LEFT_DAC_VOLUME_LDACVOL_MASK            (0xFF << LEFT_DAC_VOLUME_LDACVOL_OFFSET)

//Right DAC volume
#define RIGHT_DAC_VOLUME_DACVU_OFFSET           8
#define RIGHT_DAC_VOLUME_RDACVOL_OFFSET         0

#define RIGHT_DAC_VOLUME_DACVU_MASK             (0x01 << RIGHT_DAC_VOLUME_DACVU_OFFSET)
#define RIGHT_DAC_VOLUME_RDACVOL_MASK           (0xFF << RIGHT_DAC_VOLUME_RDACVOL_OFFSET)

//3D control
#define FILTER_3D_CONTROL_3DUC_OFFSET           6
#define FILTER_3D_CONTROL_3DLC_OFFSET           5
#define FILTER_3D_CONTROL_3DDEPTH_OFFSET        1
#define FILTER_3D_CONTROL_3DEN_OFFSET           0

#define FILTER_3D_CONTROL_3DUC_MASK             (0x01 << FILTER_3D_CONTROL_3DUC_OFFSET)
#define FILTER_3D_CONTROL_3DLC_MASK             (0x01 << FILTER_3D_CONTROL_3DLC_OFFSET)
#define FILTER_3D_CONTROL_3DDEPTH_MASK          (0x0F << FILTER_3D_CONTROL_3DDEPTH_OFFSET)
#define FILTER_3D_CONTROL_3DEN_MASK             (0x01 << FILTER_3D_CONTROL_3DUC_OFFSET)

//ALC1
#define ALC1_ALCSEL_OFFSET                      7
#define ALC1_MAXGAIN_OFFSET                     4
#define ALC1_ALCL_OFFSET                        0

#define ALC1_ALCSEL_MASK                        (0x03 << ALC1_ALCSEL_OFFSET)
#define ALC1_MAXGAIN_MASK                       (0x07 << ALC1_MAXGAIN_OFFSET)
#define ALC1_ALCL_MASK                          (0x0F << ALC1_ALCL_OFFSET)

//ACL2
#define ALC2_MINGAIN_OFFSET                     4
#define ALC2_HLD_OFFSET                         0

#define ALC2_MINGAIN_MASK                       (0x07 << ALC2_MINGAIN_OFFSET)
#define ALC2_HLD_MASK                           (0x0F << ALC2_HLD_OFFSET)

//ACL3
#define ALC3_ALCMODE_OFFSET                     8
#define ALC3_DCY_OFFSET                         4
#define ALC3_ATK_OFFSET                         0

#define ALC3_ALCMODE_MASK                       (0x01 << ALC3_ALCMODE_OFFSET)
#define ALC3_DCY_MASK                           (0x0F << ALC3_DCY_OFFSET)
#define ALC3_ATK_MASK                           (0x0F << ALC3_ATK_OFFSET)

//Noise Gate
#define NOISE_GATE_NGTH_OFFSET                  3
#define NOISE_GATE_NGAT_OFFSET                  0

#define NOISE_GATE_NGTH_MASK                    (0x1F << NOISE_GATE_NGTH_OFFSET)
#define NOISE_GATE_NGAT_MASK                    (0x01 << NOISE_GATE_NGAT_OFFSET)

//Left ADC volume
#define LEFT_ADC_VOLUME_ADCVU_OFFSET            8
#define LEFT_ADC_VOLUME_LADCVOL_OFFSET          0

#define LEFT_ADC_VOLUME_ADCVU_MASK              (0x01 << LEFT_ADC_VOLUME_ADCVU_OFFSET)
#define LEFT_ADC_VOLUME_LADCVOL_MASK            (0xFF << LEFT_ADC_VOLUME_LADCVOL_OFFSET)

//Right ADC volume
#define RIGHT_ADC_VOLUME_ADCVU_OFFSET           8
#define RIGHT_ADC_VOLUME_RADCVOL_OFFSET         0

#define RIGHT_ADC_VOLUME_ADCVU_MASK             (0x01 << RIGHT_ADC_VOLUME_ADCVU_OFFSET)
#define RIGHT_ADC_VOLUME_RADCVOL_MASK           (0xFF << RIGHT_ADC_VOLUME_RADCVOL_OFFSET)

//Additional control 1 ~ 4
#define ADDITIONAL_CONTROL1_TSDEN_OFFSET        8
#define ADDITIONAL_CONTROL1_VSEL_OFFSET         6
#define ADDITIONAL_CONTROL1_DMONOMIX_OFFSET     4
#define ADDITIONAL_CONTROL1_DATSEL_OFFSET       2
#define ADDITIONAL_CONTROL1_TOCLKSEL_OFFSET     1
#define ADDITIONAL_CONTROL1_TOEN_OFFSET         0

#define ADDITIONAL_CONTROL1_TSDEN_MASK          (0x01 << ADDITIONAL_CONTROL1_TSDEN_OFFSET)
#define ADDITIONAL_CONTROL1_VSEL_MASK           (0x03 << ADDITIONAL_CONTROL1_VSEL_OFFSET)
#define ADDITIONAL_CONTROL1_DMONOMIX_MASK       (0x01 << ADDITIONAL_CONTROL1_DMONOMIX_OFFSET)
#define ADDITIONAL_CONTROL1_DATSEL_MASK         (0x03 << ADDITIONAL_CONTROL1_DATSEL_OFFSET)
#define ADDITIONAL_CONTROL1_TOCLKSEL_MASK       (0x01 << ADDITIONAL_CONTROL1_TOCLKSEL_OFFSET)
#define ADDITIONAL_CONTROL1_TOEN_MASK           (0x01 << ADDITIONAL_CONTROL1_TOEN_OFFSET)

#define ADDITIONAL_CONTROL2_HPSWEN_OFFSET       6
#define ADDITIONAL_CONTROL2_HPSWPOL_OFFSET      5
#define ADDITIONAL_CONTROL2_TRIS_OFFSET         3
#define ADDITIONAL_CONTROL2_LRCM_OFFSET         2

#define ADDITIONAL_CONTROL2_HPSWEN_MASK         (0x01 << ADDITIONAL_CONTROL2_HPSWEN_OFFSET)
#define ADDITIONAL_CONTROL2_HPSWPOL_MASK        (0x01 << ADDITIONAL_CONTROL2_HPSWPOL_OFFSET)
#define ADDITIONAL_CONTROL2_TRIS_MASK           (0x01 << ADDITIONAL_CONTROL2_TRIS_OFFSET)
#define ADDITIONAL_CONTROL2_LRCM_MASK           (0x01 << ADDITIONAL_CONTROL2_LRCM_OFFSET)

#define ADDITIONAL_CONTROL3_VROI_OFFSET         6
#define ADDITIONAL_CONTROL3_OUT3CAP_OFFSET      3
#define ADDITIONAL_CONTROL3_ADC_ALC_SR_OFFSET   0

#define ADDITIONAL_CONTROL3_VROI_MASK           (0x01 << ADDITIONAL_CONTROL3_VROI_OFFSET)
#define ADDITIONAL_CONTROL3_OUT3CAP_MASK        (0x01 << ADDITIONAL_CONTROL3_OUT3CAP_OFFSET)
#define ADDITIONAL_CONTROL3_ADC_ALC_SR_MASK     (0x07 << ADDITIONAL_CONTROL3_ADC_ALC_SR_OFFSET)

#define ADDITIONAL_CONTROL4_GPIOPOL_OFFSET      7
#define ADDITIONAL_CONTROL4_GPIOSEL_OFFSET      4
#define ADDITIONAL_CONTROL4_HPSEL_OFFSET        2
#define ADDITIONAL_CONTROL4_TSENSEN_OFFSET      1
#define ADDITIONAL_CONTROL4_MBSEL_OFFSET        0

#define ADDITIONAL_CONTROL4_GPIOPOL_MASK        (0x01 << ADDITIONAL_CONTROL4_GPIOPOL_OFFSET)
#define ADDITIONAL_CONTROL4_GPIOSEL_MASK        (0x07 << ADDITIONAL_CONTROL4_GPIOSEL_OFFSET)
#define ADDITIONAL_CONTROL4_HPSEL_MASK          (0x03 << ADDITIONAL_CONTROL4_HPSEL_OFFSET)
#define ADDITIONAL_CONTROL4_TSENSEN_MASK        (0x01 << ADDITIONAL_CONTROL4_TSENSEN_OFFSET)
#define ADDITIONAL_CONTROL4_MBSEL_MASK          (0x01 << ADDITIONAL_CONTROL4_MBSEL_OFFSET)

//Pwr Mgmt 1 ~ 3
#define PWR_MGMT1_VMIDSEL_OFFSET                7
#define PWR_MGMT1_VREF_OFFSET                   6
#define PWR_MGMT1_AINL_OFFSET                   5
#define PWR_MGMT1_AINR_OFFSET                   4
#define PWR_MGMT1_ADCL_OFFSET                   3
#define PWR_MGMT1_ADCR_OFFSET                   2
#define PWR_MGMT1_MICB_OFFSET                   1
#define PWR_MGMT1_DIGENB_OFFSET                 0

#define PWR_MGMT1_VMIDSEL_MASK                  (0x03 << PWR_MGMT1_VMIDSEL_OFFSET)
#define PWR_MGMT1_VREF_MASK                     (0x01 << PWR_MGMT1_VREF_OFFSET)
#define PWR_MGMT1_AINL_MASK                     (0x01 << PWR_MGMT1_AINL_OFFSET)
#define PWR_MGMT1_AINR_MASK                     (0x01 << PWR_MGMT1_AINR_OFFSET)
#define PWR_MGMT1_ADCL_MASK                     (0x01 << PWR_MGMT1_ADCL_OFFSET)
#define PWR_MGMT1_ADCR_MASK                     (0x01 << PWR_MGMT1_ADCR_OFFSET)
#define PWR_MGMT1_MICB_MASK                     (0x01 << PWR_MGMT1_MICB_OFFSET)
#define PWR_MGMT1_DIGENB_MASK                   (0x01 << PWR_MGMT1_DIGENB_OFFSET)

#define PWR_MGMT2_DACL_OFFSET                   8
#define PWR_MGMT2_DACR_OFFSET                   7
#define PWR_MGMT2_LOUT1_OFFSET                  6
#define PWR_MGMT2_ROUT1_OFFSET                  5
#define PWR_MGMT2_SPKL_OFFSET                   4
#define PWR_MGMT2_SPKR_OFFSET                   3
#define PWR_MGMT2_OUT3_OFFSET                   1
#define PWR_MGMT2_PLL_EN_OFFSET                 0

#define PWR_MGMT2_DACL_MASK                     (0x01 << PWR_MGMT2_DACL_OFFSET)
#define PWR_MGMT2_DACR_MASK                     (0x01 << PWR_MGMT2_DACR_OFFSET)
#define PWR_MGMT2_LOUT1_MASK                    (0x01 << PWR_MGMT2_LOUT1_OFFSET)
#define PWR_MGMT2_ROUT1_MASK                    (0x01 << PWR_MGMT2_ROUT1_OFFSET)
#define PWR_MGMT2_SPKL_MASK                     (0x01 << PWR_MGMT2_SPKL_OFFSET)
#define PWR_MGMT2_SPKR_MASK                     (0x01 << PWR_MGMT2_SPKR_OFFSET)
#define PWR_MGMT2_OUT3_MASK                     (0x01 << PWR_MGMT2_OUT3_OFFSET)
#define PWR_MGMT2_PLL_EN_MASK                   (0x01 << PWR_MGMT2_PLL_EN_OFFSET)

#define PWR_MGMT3_LMIC_OFFSET                   5
#define PWR_MGMT3_RMIC_OFFSET                   4
#define PWR_MGMT3_LOMIX_OFFSET                  3
#define PWR_MGMT3_ROMIX_OFFSET                  2

#define PWR_MGMT3_LMIC_MASK                     (0x01 << PWR_MGMT3_LMIC_OFFSET)
#define PWR_MGMT3_RMIC_MASK                     (0x01 << PWR_MGMT3_RMIC_OFFSET)
#define PWR_MGMT3_LOMIX_MASK                    (0x01 << PWR_MGMT3_LOMIX_OFFSET)
#define PWR_MGMT3_ROMIX_MASK                    (0x01 << PWR_MGMT3_ROMIX_OFFSET)

//Anti-pop 1 ~ 2
#define ANTI_POP1_POBCTRL_OFFSET                7
#define ANTI_POP1_BUFDCOPEN_OFFSET              4
#define ANTI_POP1_BUFIOEN_OFFSET                3
#define ANTI_POP1_SOFT_ST_OFFSET                2
#define ANTI_POP1_HPSTBY_OFFSET                 0

#define ANTI_POP1_POBCTRL_MASK                  (0x01 << ANTI_POP1_POBCTRL_OFFSET)
#define ANTI_POP1_BUFDCOPEN_MASK                (0x01 << ANTI_POP1_BUFDCOPEN_OFFSET)
#define ANTI_POP1_BUFIOEN_MASK                  (0x01 << ANTI_POP1_BUFIOEN_OFFSET)
#define ANTI_POP1_SOFT_ST_MASK                  (0x01 << ANTI_POP1_SOFT_ST_OFFSET)
#define ANTI_POP1_HPSTBY_MASK                   (0x01 << ANTI_POP1_HPSTBY_OFFSET)

#define ANTI_POP2_DISOP_OFFSET                  6
#define ANTI_POP2_DRES_OFFSET                   4

#define ANTI_POP2_DISOP_MASK                    (0x01 << ANTI_POP2_DISOP_OFFSET)
#define ANTI_POP2_DRES_MASK                     (0x03 << ANTI_POP2_DRES_OFFSET)

//ADCL signal path
#define ADCL_SIGNAL_PATH_LMN1_OFFSET            8
#define ADCL_SIGNAL_PATH_LMP3_OFFSET            7
#define ADCL_SIGNAL_PATH_LMP2_OFFSET            6
#define ADCL_SIGNAL_PATH_LMICBOOST_OFFSET       4
#define ADCL_SIGNAL_PATH_LMIC2B_OFFSET          3

#define ADCL_SIGNAL_PATH_LMN1_MASK              (0x01 << ADCL_SIGNAL_PATH_LMN1_OFFSET)
#define ADCL_SIGNAL_PATH_LMP3_MASK              (0x01 << ADCL_SIGNAL_PATH_LMP3_OFFSET)
#define ADCL_SIGNAL_PATH_LMP2_MASK              (0x01 << ADCL_SIGNAL_PATH_LMP2_OFFSET)
#define ADCL_SIGNAL_PATH_LMICBOOST_MASK         (0x03 << ADCL_SIGNAL_PATH_LMICBOOST_OFFSET)
#define ADCL_SIGNAL_PATH_LMIC2B_MASK            (0x01 << ADCL_SIGNAL_PATH_LMIC2B_OFFSET)

//ADCR signal path
#define ADCR_SIGNAL_PATH_RMN1_OFFSET            8
#define ADCR_SIGNAL_PATH_RMP3_OFFSET            7
#define ADCR_SIGNAL_PATH_RMP2_OFFSET            6
#define ADCR_SIGNAL_PATH_RMICBOOST_OFFSET       4
#define ADCR_SIGNAL_PATH_RMIC2B_OFFSET          3

#define ADCR_SIGNAL_PATH_RMN1_MASK              (0x01 << ADCR_SIGNAL_PATH_RMN1_OFFSET)
#define ADCR_SIGNAL_PATH_RMP3_MASK              (0x01 << ADCR_SIGNAL_PATH_RMP3_OFFSET)
#define ADCR_SIGNAL_PATH_RMP2_MASK              (0x01 << ADCR_SIGNAL_PATH_RMP2_OFFSET)
#define ADCR_SIGNAL_PATH_RMICBOOST_MASK         (0x01 << ADCR_SIGNAL_PATH_RMICBOOST_OFFSET)
#define ADCR_SIGNAL_PATH_RMIC2B_MASK            (0x01 << ADCR_SIGNAL_PATH_RMIC2B_OFFSET)

//Left out Mix
#define LEFT_OUT_MIX1_LD2LO_OFFSET              8
#define LEFT_OUT_MIX1_LI2LO_OFFSET              7
#define LEFT_OUT_MIX1_LI2LOVOL_OFFSET           4

#define LEFT_OUT_MIX1_LD2LO_MASK                (0x01 << LEFT_OUT_MIX1_LD2LO_OFFSET)
#define LEFT_OUT_MIX1_LI2LO_MASK                (0x01 << LEFT_OUT_MIX1_LI2LO_OFFSET)
#define LEFT_OUT_MIX1_LI2LOVOL_MASK             (0x07 << LEFT_OUT_MIX1_LI2LOVOL_OFFSET)

//Right out Mix
#define RIGHT_OUT_MIX2_RD2RO_OFFSET             8
#define RIGHT_OUT_MIX2_RI2RO_OFFSET             7
#define RIGHT_OUT_MIX2_RI2ROVOL_OFFSET          4

#define RIGHT_OUT_MIX2_RD2RO_MASK               (0x01 << RIGHT_OUT_MIX2_RD2RO_OFFSET)
#define RIGHT_OUT_MIX2_RI2RO_MASK               (0x01 << RIGHT_OUT_MIX2_RI2RO_OFFSET)
#define RIGHT_OUT_MIX2_RI2ROVOL_MASK            (0x07 << RIGHT_OUT_MIX2_RI2ROVOL_OFFSET)

//Mono out Mix 1 ~ 2
#define MONO_OUT_MIX1_L2MO_OFFSET               7

#define MONO_OUT_MIX1_L2MO_MASK                 (0x01 << MONO_OUT_MIX1_L2MO_OFFSET)

#define MONO_OUT_MIX2_R2MO_OFFSET               7

#define MONO_OUT_MIX2_R2MO_MASK                 (0x01 << MONO_OUT_MIX2_R2MO_OFFSET)

//LOUT2 volume
#define LOUT2_VOLUME_SPKVU_OFFSET               8
#define LOUT2_VOLUME_SPKLZC_OFFSET              7
#define LOUT2_VOLUME_SPKLVOL_OFFSET             0

#define LOUT2_VOLUME_SPKVU_MASK                 (0x01 << LOUT2_VOLUME_SPKVU_OFFSET)
#define LOUT2_VOLUME_SPKLZC_MASK                (0x01 << LOUT2_VOLUME_SPKLZC_OFFSET)
#define LOUT2_VOLUME_SPKLVOL_MASK               (0x7F << LOUT2_VOLUME_SPKLVOL_OFFSET)

//ROUT2 volume
#define ROUT2_VOLUME_SPKVU_OFFSET               8
#define ROUT2_VOLUME_SPKRZC_OFFSET              7
#define ROUT2_VOLUME_SPKRVOL_OFFSET             0

#define ROUT2_VOLUME_SPKVU_MASK                 (0x01 << ROUT2_VOLUME_SPKVU_OFFSET)
#define ROUT2_VOLUME_SPKRZC_MASK                (0x01 << ROUT2_VOLUME_SPKRZC_OFFSET)
#define ROUT2_VOLUME_SPKRVOL_MASK               (0x7F << ROUT2_VOLUME_SPKRVOL_OFFSET)

//MONOOUT volume
#define MONOOUT_VOLUME_MOUTVOL_OFFSET           6

#define MONOOUT_VOLUME_MOUTVOL_MASK             (0x01 << MONOOUT_VOLUME_MOUTVOL_OFFSET)

//Input boost mixer 1 ~ 2
#define INPUT_BOOST_MIXER1_LIN3BOOST_OFFSET     4
#define INPUT_BOOST_MIXER1_LIN2BOOST_OFFSET     1

#define INPUT_BOOST_MIXER1_LIN3BOOST_MASK       (0x07 << INPUT_BOOST_MIXER1_LIN3BOOST_OFFSET)
#define INPUT_BOOST_MIXER1_LIN2BOOST_MASK       (0x07 << INPUT_BOOST_MIXER1_LIN2BOOST_OFFSET)

#define INPUT_BOOST_MIXER2_RIN3BOOST_OFFSET     4
#define INPUT_BOOST_MIXER2_RIN2BOOST_OFFSET     1

#define INPUT_BOOST_MIXER2_RIN3BOOST_MASK       (0x07 << INPUT_BOOST_MIXER2_RIN3BOOST_OFFSET)
#define INPUT_BOOST_MIXER2_RIN2BOOST_MASK       (0x07 << INPUT_BOOST_MIXER2_RIN2BOOST_OFFSET)

//Bypass 1 ~ 2
#define BYPASS1_LB2LO_OFFSET                    7
#define BYPASS1_LB2LOVOL_OFFSET                 4

#define BYPASS1_LB2LO_MASK                      (0x01 << BYPASS1_LB2LO_OFFSET)
#define BYPASS1_LB2LOVOL_MASK                   (0x07 << BYPASS1_LB2LOVOL_OFFSET)

#define BYPASS2_RB2RO_OFFSET                    7
#define BYPASS2_RB2ROVOL_OFFSET                 4

#define BYPASS2_RB2RO_MASK                      (0x01 << BYPASS2_RB2RO_OFFSET)
#define BYPASS2_RB2ROVOL_MASK                   (0x07 << BYPASS2_RB2ROVOL_OFFSET)

//Class D Control 1, 3
#define CLASS_D_CONTROL1_SPK_OP_EN_OFFSET       6

#define CLASS_D_CONTROL1_SPK_OP_EN_MASK         (0x03 << CLASS_D_CONTROL1_SPK_OP_EN_OFFSET)

#define CLASS_D_CONTROL3_DCGAIN_OFFSET          3
#define CLASS_D_CONTROL3_ACGAIN_OFFSET          0

#define CLASS_D_CONTROL3_DCGAIN_MASK            (0x07 << CLASS_D_CONTROL3_DCGAIN_OFFSET)
#define CLASS_D_CONTROL3_ACGAIN_MASK            (0x07 << CLASS_D_CONTROL3_ACGAIN_OFFSET)

//PLL N
#define PLL_N_OPCLKDIV_OFFSET                   6
#define PLL_N_SDM_OFFSET                        5
#define PLL_N_PLLRESCALE_OFFSET                 4
#define PLL_N_PLLN_OFFSET                       0

#define PLL_N_OPCLKDIV_MASK                     (0x07 << PLL_N_OPCLKDIV_OFFSET)
#define PLL_N_SDM_MASK                          (0x01 << PLL_N_SDM_OFFSET)
#define PLL_N_PLLRESCALE_MASK                   (0x01 << PLL_N_PLLRESCALE_OFFSET)
#define PLL_N_PLLN_MASK                         (0x0F << PLL_N_PLLN_OFFSET)

//PLL K 1 ~ 3
#define PLL_K1_PLLK_OFFSET                      0
#define PLL_K2_PLLK_OFFSET                      0
#define PLL_K3_PLLK_OFFSET                      0

#define PLL_K1_PLLK_MASK                        (0xFF << PLL_K1_PLLK_OFFSET)
#define PLL_K2_PLLK_MASK                        (0xFF << PLL_K2_PLLK_OFFSET)
#define PLL_K3_PLLK_MASK                        (0xFF << PLL_K3_PLLK_OFFSET)


/* 
 * Enum table *****************************************************************
 */


// Left input volume
enum 
{
    LEFT_INPUT_VOLUME_IPVU          = 0x01,
    LEFT_INPUT_VOLUME_LINMUTE       = 0x01,
    LEFT_INPUT_VOLUME_LIZC          = 0x01,
    LEFT_INPUT_VOLUME_LINVOL        = 0x3F
};

// Right input volume
enum 
{
    RIGHT_INPUT_VOLUME_IPVU         = 0x01,
    RIGHT_INPUT_VOLUME_RINMUTE      = 0x01,
    RIGHT_INPUT_VOLUME_RIZC         = 0x01,
    RIGHT_INPUT_VOLUME_RINVOL       = 0x3F
};

// LOUT1 volume
enum 
{
    LOUT1_VOLUME_OUT1VU             = 0x01,
    LOUT1_VOLUME_LO1ZC              = 0x01,
    LOUT1_VOLUME_LOUT1VOL           = 0x7F
};

// ROUT1 volume
enum 
{
    ROUT1_VOLUME_OUT1VU             = 0x01,
    ROUT1_VOLUME_RO1ZC              = 0x01,
    ROUT1_VOLUME_ROUT1VOL           = 0x7F
};

// Clocking 1 ~ 2
enum 
{
    CLOCKING1_ADCDIV                = 0x07,
    CLOCKING1_DACDIV                = 0x07,
    CLOCKING1_SYSCLKDIV             = 0x03, 
    CLOCKING1_CLKSEL                = 0x01 
};

enum 
{
    CLOCKING2_DCLKDIV               = 0x07,
    CLOCKING2_BCLKDIV               = 0x0F
};

// ADC & DAC Control 1 ~ 2
enum 
{
    ADC_DAC_CONTROL1_DACDIV2        = 0x01,
    ADC_DAC_CONTROL1_ADCPOL         = 0x03,
    ADC_DAC_CONTROL1_DACMU          = 0x01,
    ADC_DAC_CONTROL1_DEEMPH         = 0x03,
    ADC_DAC_CONTROL1_ADCHPD         = 0x01
};

enum 
{
    ADC_DAC_CONTROL2_DACPOL         = 0x03,
    ADC_DAC_CONTROL2_DACSMM         = 0x01,
    ADC_DAC_CONTROL2_DACMR          = 0x01,
    ADC_DAC_CONTROL2_DACSLOPE       = 0x01
};

// Audio Interface 1 ~ 2
enum 
{
    AUDIO_INTERFACE1_ALRSWAP        = 0x01,
    AUDIO_INTERFACE1_BCLKINV        = 0x01,
    AUDIO_INTERFACE1_MS             = 0x01,
    AUDIO_INTERFACE1_DLRSWAP        = 0x01,
    AUDIO_INTERFACE1_LRP            = 0x01,
    AUDIO_INTERFACE1_WL             = 0x03,
    AUDIO_INTERFACE1_FORMAT         = 0x03
};

enum 
{
    AUDIO_INTERFACE2_ALRCGPIO       = 0x01,
    AUDIO_INTERFACE2_WL8            = 0x01,
    AUDIO_INTERFACE2_DACCOMP        = 0x03,
    AUDIO_INTERFACE2_ADCCOMP        = 0x03,
    AUDIO_INTERFACE2_LOOPBACK       = 0x01
};

// Left DAC volume
enum 
{
    LEFT_DAC_VOLUME_DACVU           = 0x01,
    LEFT_DAC_VOLUME_LDACVOL         = 0xFF
};

// Right DAC volume
enum 
{
    RIGHT_DAC_VOLUME_DACVU          = 0x01,
    RIGHT_DAC_VOLUME_RDACVOL        = 0xFF
};

// 3D control
enum 
{
    FILTER_3D_CONTROL_3DUC          = 0x01,
    FILTER_3D_CONTROL_3DLC          = 0x01,
    FILTER_3D_CONTROL_3DDEPTH       = 0x0F,
    FILTER_3D_CONTROL_3DEN          = 0x01
};

// ALC 1 ~ 3
enum 
{
    ALC1_ALCSEL                     = 0x03,
    ALC1_MAXGAIN                    = 0x07,
    ALC1_ALCL                       = 0x0F
};

enum 
{
    ALC2_MINGAIN                    = 0x07,
    ALC2_HLD                        = 0x0F
};

enum 
{
    ALC3_ALCMODE                    = 0x01,
    ALC3_DCY                        = 0x0F,
    ALC3_ATK                        = 0x0F
};

// Noise gate
enum 
{
    NOISE_GATE_NGTH                 = 0x1F,
    NOISE_GATE_NGAT                 = 0x01
};

// Left ADC volume
enum 
{
    LEFT_ADC_VOLUME_ADCVU           = 0x01,
    LEFT_ADC_VOLUME_LADCVOL         = 0xFF
};

// Right ADC volume
enum 
{
    RIGHT_ADC_VOLUME_ADCVU          = 0x01,
    RIGHT_ADC_VOLUME_RADCVOL        = 0xFF
};

// Additional control 1 ~ 4
enum 
{
    ADDITIONAL_CONTROL1_TSDEN       = 0x01,
    ADDITIONAL_CONTROL1_VSEL        = 0x03,
    ADDITIONAL_CONTROL1_DMONOMIX    = 0x01,
    ADDITIONAL_CONTROL1_DATSEL      = 0x03,
    ADDITIONAL_CONTROL1_TOCLKSEL    = 0x01,
    ADDITIONAL_CONTROL1_TOEN        = 0X01
};

enum 
{
    ADDITIONAL_CONTROL2_HPSWEN      = 0x01,
    ADDITIONAL_CONTROL2_HPSWPOL     = 0x01,
    ADDITIONAL_CONTROL2_TRIS        = 0x01,
    ADDITIONAL_CONTROL2_LRCM        = 0x01
};

enum 
{
    ADDITIONAL_CONTROL3_VROI        = 0x01,
    ADDITIONAL_CONTROL3_OUT3CAP     = 0x01,
    ADDITIONAL_CONTROL3_ADC_ALC_SR  = 0x07
};

enum 
{
    ADDITIONAL_CONTROL4_GPIOPOL     = 0x01,
    ADDITIONAL_CONTROL4_GPIOSEL     = 0x07,
    ADDITIONAL_CONTROL4_HPSEL       = 0x03,
    ADDITIONAL_CONTROL4_TSENSEN     = 0x01,
    ADDITIONAL_CONTROL4_MBSEL       = 0x01
};

// Pwr mgmt 1 ~ 3
enum 
{
    PWR_MGMT1_VMIDSEL               = 0x03,
    PWR_MGMT1_VREF                  = 0x01,
    PWR_MGMT1_AINL                  = 0x01,
    PWR_MGMT1_AINR                  = 0x01,
    PWR_MGMT1_ADCL                  = 0x01,
    PWR_MGMT1_ADCR                  = 0x01,
    PWR_MGMT1_MICB                  = 0x01,
    PWR_MGMT1_DIGENB                = 0x01
};

enum 
{
    PWR_MGMT2_DACL                  = 0x01,
    PWR_MGMT2_DACR                  = 0x01,
    PWR_MGMT2_LOUT1                 = 0x01,
    PWR_MGMT2_ROUT1                 = 0x01,
    PWR_MGMT2_SPKL                  = 0x01,
    PWR_MGMT2_SPKR                  = 0x01,
    PWR_MGMT2_OUT3                  = 0x01,
    PWR_MGMT2_PLL_EN                = 0x01
};

enum 
{
    PWR_MGMT3_LMIC                  = 0x01,
    PWR_MGMT3_RMIC                  = 0x01,
    PWR_MGMT3_LOMIX                 = 0x01,
    PWR_MGMT3_ROMIX                 = 0x01
};

// Anti-pop 1 ~ 2
enum 
{
    ANTI_POP1_POBCTRL               = 0x01,
    ANTI_POP1_BUFDCOPEN             = 0x01,
    ANTI_POP1_BUFIOEN               = 0x01,
    ANTI_POP1_SOFT_ST               = 0x01,
    ANTI_POP1_HPSTBY                = 0x01
};

enum 
{
    ANTI_POP2_DISOP                 = 0x01,
    ANTI_POP2_DRES                  = 0x03
};

// ADCL signal path
enum 
{
    ADCL_SIGNAL_PATH_LMN1           = 0x01,
    ADCL_SIGNAL_PATH_LMP3           = 0x01,
    ADCL_SIGNAL_PATH_LMP2           = 0x01,
    ADCL_SIGNAL_PATH_LMICBOOST      = 0x03,
    ADCL_SIGNAL_PATH_LMIC2B         = 0x01
};

// ADCR signal path
enum 
{
    ADCR_SIGNAL_PATH_RMN1           = 0x01,
    ADCR_SIGNAL_PATH_RMP3           = 0x01,
    ADCR_SIGNAL_PATH_RMP2           = 0x01,
    ADCR_SIGNAL_PATH_RMICBOOST      = 0x03,
    ADCR_SIGNAL_PATH_RMIC2B         = 0x01
};

// Left out Mix1
enum 
{
    LEFT_OUT_MIX1_LD2LO             = 0x01,
    LEFT_OUT_MIX1_LI2LO             = 0x01,
    LEFT_OUT_MIX1_LI2LOVOL          = 0x07
};

// Right out Mix2
enum 
{
    RIGHT_OUT_MIX2_RD2RO            = 0x01,
    RIGHT_OUT_MIX2_RI2RO            = 0x01,
    RIGHT_OUT_MIX2_RI2ROVOL         = 0x07
};

// Mono out Mix 1, 2
enum 
{
    MONO_OUT_MIX1_L2MO              = 0x01,
    MONO_OUT_MIX2_R2MO              = 0x01
};

// LOUT2 volume
enum 
{
    LOUT2_VOLUME_SPKVU              = 0x01,
    LOUT2_VOLUME_SPKLZC             = 0x01,
    LOUT2_VOLUME_SPKLVOL            = 0x7F
};

// ROUT2 volume
enum 
{
    ROUT2_VOLUME_SPKVU              = 0x01,
    ROUT2_VOLUME_SPKRZC             = 0x01,
    ROUT2_VOLUME_SPKRVOL            = 0x7F
};

// MONOOUT volume
enum 
{
    MONOOUT_VOLUME_MOUTVOL          = 0x01
};

// Input boost mixer 1 ~ 2
enum 
{
    INPUT_BOOST_MIXER1_LIN3BOOST    = 0x07,
    INPUT_BOOST_MIXER1_LIN2BOOST    = 0x07
};

enum 
{
    INPUT_BOOST_MIXER2_RIN3BOOST    = 0x07,
    INPUT_BOOST_MIXER2_RIN2BOOST    = 0x07
};

// Bypass 1 ~ 2
enum 
{
    BYPASS1_LB2LO                   = 0x01,
    BYPASS1_LB2LOVOL                = 0x07
};

enum
{
    BYPASS2_RB2RO                   = 0x01,
    BYPASS2_RB2ROVOL                = 0x07
};

// Class D Control 1, 3
enum
{
    CLASS_D_CONTROL1_SPK_OP_EN      = 0x03
};

enum 
{
    CLASS_D_CONTROL3_DCGAIN         = 0x07,
    CLASS_D_CONTROL3_ACGAIN         = 0x07
};

// PLL N
enum 
{
    PLL_N_OPCLKDIV                  = 0x07,
    PLL_N_SDM                       = 0x01,
    PLL_N_PLLRESCALE                = 0x01,
    PLL_N_PLLN                      = 0x0F
};

// PLL K1 ~ K3
enum 
{
    PLL_K1_PLLK                     = 0xFF,
    PLL_K2_PLLK                     = 0xFF,
    PLL_K3_PLLK                     = 0xFF
};

/* 
 * Enum table (Register setting value) *****************************************************************
 */

//additional_control4 - HPSEL
enum
{
    JACK_DETECT_INPUT_GPIO1         = 0x00,
    JACK_DETECT_INPUT_JD2           = 0x02,
    JACK_DETECT_INPUT_JD3           = 0x03
};

//additional_control2 - HPSWPOL
enum
{
    HPDETECT_HIGH_HDPHONE           = 0x00,
    HPDETECT_HIGH_SPEAKER           = 0x01
};

//additional_control2 - HPSWEN
enum
{
    HPSWEN_DISABLE                  = 0x00,
    HPSWEN_ENABLE                   = 0x01
};

//additional_control1 - TSDEN
enum
{
    THERMAL_SHUTDOWN_DISABLE        = 0x00,
    THERMAL_SHUTDOWN_ENABLE         = 0x01
};

//audio_interface1 - MS
enum
{
    AUDIO_CODEC_SLAVE               = 0x00,
    AUDIO_CODEC_MASTER              = 0x01
};

///audio_interface1 - WL (Audio Data Word Length)
enum
{
    AUDIO_WORD_LENGTH_16bit         = 0x00,
    AUDIO_WORD_LENGTH_20bit         = 0x01,
    AUDIO_WORD_LENGTH_24bit         = 0x10,
    AUDIO_WORD_LENGTH_32bit         = 0x11
};

// ADC, DAC sampling rate
enum 
{
    SAMPLING_RATE_8K,
    SAMPLING_RATE_12k,
    SAMPLING_RATE_16K,  
    SAMPLING_RATE_24K,
    SAMPLING_RATE_32K,  
    SAMPLING_RATE_44K,     // require PLL_CLK 11.2896
    SAMPLING_RATE_48K,     // require PLL_CLK 12.288M
    SAMPLING_RATE_NUM,
};
typedef uint8_t sample_rate_t;

// PLL CLOCK mode
enum 
{
    PLL_CLK_MODE_12_288M,
    PLL_CLK_MODE_11_289M
};

enum 
{
    DIVIDE_SYSCLK_BY1               = 0x00,
    DIVIDE_SYSCLK_BY2               = 0x02
};

// DAC Mono Mix
enum 
{
    DAC_STEREO                      = 0x00,
    DAC_MONO                        = 0x01
};

enum 
{
    LEFT_RIGHT                      = 0x00,
    LEFT_LEFT                       = 0x01,
    RIGHT_RIGHT                     = 0x02,
    RIGHT_LEFT                      = 0x03
};

enum 
{
    OUTPUT_AMP_GAIN_MUTE            = 0x00,
    OUTPUT_AMP_GAIN_MINUS_72db      = 0x31,
    OUTPUT_AMP_GAIN_MINUS_60db      = 0x3D,
    OUTPUT_AMP_GAIN_MINUS_48db      = 0x49,
    OUTPUT_AMP_GAIN_MINUS_42db      = 0x4E,
    OUTPUT_AMP_GAIN_MINUS_36db      = 0x55,
    OUTPUT_AMP_GAIN_MINUS_30db      = 0x5B,
    OUTPUT_AMP_GAIN_MINUS_24db      = 0x61,
    OUTPUT_AMP_GAIN_MINUS_18db      = 0x67,
    OUTPUT_AMP_GAIN_MINUS_12db      = 0x6D,
    OUTPUT_AMP_GAIN_MINUS_10db      = 0x6F,
    OUTPUT_AMP_GAIN_MINUS_8db       = 0x71,
    OUTPUT_AMP_GAIN_MINUS_6db       = 0x73,
    OUTPUT_AMP_GAIN_MINUS_4db       = 0x75,
    OUTPUT_AMP_GAIN_MINUS_2db       = 0x77,
    OUTPUT_AMP_GAIN_0db             = 0x79,
    OUTPUT_AMP_GAIN_2db             = 0x7B,
    OUTPUT_AMP_GAIN_4db             = 0x7D,
    OUTPUT_AMP_GAIN_6db             = 0x7F
};
typedef uint8_t O_AMP_GAIN_T;


enum 
{
    OUTPUT_DIGITAL_VOL_MUTE         = 0x00,
    OUTPUT_DIGITAL_VOL_MINUS_127db  = 0x01,
    OUTPUT_DIGITAL_VOL_MINUS_120db  = 0x0F,
    OUTPUT_DIGITAL_VOL_MINUS_110db  = 0x23,
    OUTPUT_DIGITAL_VOL_MINUS_100db  = 0x37,
    OUTPUT_DIGITAL_VOL_MINUS_90db   = 0x4B,
    OUTPUT_DIGITAL_VOL_MINUS_75db   = 0x69,
    OUTPUT_DIGITAL_VOL_MINUS_60db   = 0x87,
    OUTPUT_DIGITAL_VOL_MINUS_45db   = 0xA5,
    OUTPUT_DIGITAL_VOL_MINUS_30db   = 0xC3,
    OUTPUT_DIGITAL_VOL_MINUS_15db   = 0xE1,
    OUTPUT_DIGITAL_VOL_0db          = 0xFF
};
typedef uint8_t O_DITIGAL_VOL_T;

enum 
{
    Input_AMP_MGAIN_MINUS_17_25dB   = 0x00,
    Input_AMP_MGAIN_MINUS_13_5dB    = 0x03,
    Input_AMP_MGAIN_MINUS_9dB       = 0x09,
    Input_AMP_MGAIN_MINUS_4_5dB     = 0x0F,
    Input_AMP_MGAIN_0dB             = 0x15,
    Input_AMP_MGAIN_3dB             = 0x1B,
    Input_AMP_MGAIN_7_5dB           = 0x21,
    Input_AMP_MGAIN_12dB            = 0x27,
    Input_AMP_MGAIN_16_5dB          = 0x2D,
    Input_AMP_MGAIN_21dB            = 0x33,
    Input_AMP_MGAIN_25_5dB          = 0x39,
    Input_AMP_MGAIN_30dB            = 0x3F
};
typedef uint8_t Input_AMP_MGAIN_T;

enum 
{
    INPUT_DIGITAL_VOLUME_MUTE       = 0x00,
    INPUT_DIGITAL_VOLUME_MINUS_90dB = 0x0F,
    INPUT_DIGITAL_VOLUME_MINUS_75dB = 0x2D,
    INPUT_DIGITAL_VOLUME_MINUS_60dB = 0x4B,
    INPUT_DIGITAL_VOLUME_MINUS_45dB = 0x69,
    INPUT_DIGITAL_VOLUME_MINUS_30dB = 0x87,
    INPUT_DIGITAL_VOLUME_MINUS_15dB = 0xA5,
    INPUT_DIGITAL_VOLUME_0dB        = 0xC3,
    INPUT_DIGITAL_VOLUME_15dB       = 0xE1,
    INPUT_DIGITAL_VOLUME_30dB       = 0xFF,
};
typedef uint8_t I_MIC_DVOL_T;

enum {
    IN_BOOST_MUTE                   = 0x00,
    IN_BOOST_MINUS_12db             = 0x01,
    IN_BOOST_MINUS_9db              = 0x02,
    IN_BOOST_MINUS_6db              = 0x03,
    IN_BOOST_MINUS_3db              = 0x04,
    IN_BOOST_0db                    = 0x05,
    IN_BOOST_3db                    = 0x06,
    IN_BOOST_MAX                    = 0x07
};

#endif
