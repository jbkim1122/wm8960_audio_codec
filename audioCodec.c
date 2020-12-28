
/*********************************************************************
 * INCLUDES
 */

#include "data_types.h"
#include "AudioCodec.h"


/******************************************************************************
*                         Definitions
******************************************************************************/

#ifdef CODEC_WM8960
#define I2C_SLAVE_ADDR           WM8960_Addr
#endif

/* ===============================
 * audio codec default configs 
 * ============================== */

#if defined(CODEC_WM8960)
#define MIC_AMP_GAIN_LEVEL_DEFAULT             MIC_AMP_GAIN_LEVEL7       
#define MIC_DIGITAL_VOLUME_LEVEL_DEFAULT       MIC_DIGITAL_VOLUME_LEVEL7 

#define SPK_AMP_GAIN_LEVEL_DEFAULT             SPK_AMP_GAIN_LEVEL7      
#define SPK_DIGITAL_VOLUME_LEVEL_DEFAULT       SPK_DIGITAL_VOLUME_LEVEL8 
#endif

#define INPUT_PATH_DEFAULT                     INPUT_MIC_IN
#define OUTPUT_PATH_DEFAULT                    OUTPUT_HP_OUT

/******************************************************************************
*                         Type Defines
******************************************************************************/


/******************************************************************************
*                       Variables
******************************************************************************/

/* spk amp gain table */
uint8_t spk_amp_gain_val_tbl[SPK_AMP_GAIN_LEVEL_NUM] =
{
#if defined(CODEC_WM8960)
    OUTPUT_AMP_GAIN_MUTE,            // mute
    OUTPUT_AMP_GAIN_MINUS_10db,      // -10dB
    OUTPUT_AMP_GAIN_MINUS_8db,       // -8dB
    OUTPUT_AMP_GAIN_MINUS_6db,       // -6dB
    OUTPUT_AMP_GAIN_MINUS_4db,       // -4dB
    OUTPUT_AMP_GAIN_MINUS_2db,       // -2dB
    OUTPUT_AMP_GAIN_0db,             // 0dB
    OUTPUT_AMP_GAIN_2db,             // 2dB
    OUTPUT_AMP_GAIN_4db,             // 4dB
    OUTPUT_AMP_GAIN_6db,             // 6dB
#else
    0x00
#endif
};

/* < spk digital_volume value table >*/

uint8_t spk_digital_volume_val_tbl[SPK_DIGITAL_VOLUME_LEVEL_NUM] =
{
#if defined(CODEC_WM8960)
    OUTPUT_DIGITAL_VOL_MUTE,         //  MUTE
    OUTPUT_DIGITAL_VOL_MINUS_127db,  // -127dB
    OUTPUT_DIGITAL_VOL_MINUS_100db,  // -100dB
    OUTPUT_DIGITAL_VOL_MINUS_90db,   // -90dB
    OUTPUT_DIGITAL_VOL_MINUS_75db,   // -75dB
    OUTPUT_DIGITAL_VOL_MINUS_60db,   // -60dB
    OUTPUT_DIGITAL_VOL_MINUS_45db,   // -45dB
    OUTPUT_DIGITAL_VOL_MINUS_30db,   // -30dB
    OUTPUT_DIGITAL_VOL_MINUS_15db,   // -15dB
    OUTPUT_DIGITAL_VOL_0db,          //  0dB
#else
    0x00
#endif
};

/* mic amp gain value table */
uint8_t mic_amp_gain_val_tbl[MIC_AMP_GAIN_LEVEL_NUM] =
{
#if defined(CODEC_WM8960)
    Input_AMP_MGAIN_MINUS_9dB,       // -9dB
    Input_AMP_MGAIN_MINUS_4_5dB,     // -4.5dB
    Input_AMP_MGAIN_0dB,             // 0dB
    Input_AMP_MGAIN_3dB,             // 3dB
    Input_AMP_MGAIN_7_5dB,           // 7.5dB
    Input_AMP_MGAIN_12dB,            // 12dB
    Input_AMP_MGAIN_16_5dB,          // 16.5dB
    Input_AMP_MGAIN_21dB,            // 21dB
    Input_AMP_MGAIN_25_5dB,          // 25.5dB
    Input_AMP_MGAIN_30dB,            // 30dB
#else
    0x00
#endif  
};

/* mic digital gain value table */
uint8_t mic_digital_gain_val_tbl[MIC_DIGITAL_VOLUME_LEVEL_NUM] =
{
#if defined(CODEC_WM8960)
    INPUT_DIGITAL_VOLUME_MUTE,       // mute
    INPUT_DIGITAL_VOLUME_MINUS_90dB, // -90dB
    INPUT_DIGITAL_VOLUME_MINUS_75dB, // -75dB
    INPUT_DIGITAL_VOLUME_MINUS_60dB, // -60dB
    INPUT_DIGITAL_VOLUME_MINUS_45dB, // -45dB
    INPUT_DIGITAL_VOLUME_MINUS_30dB, // -30dB
    INPUT_DIGITAL_VOLUME_MINUS_15dB, // -15dB
    INPUT_DIGITAL_VOLUME_0dB,        // 0dB
    INPUT_DIGITAL_VOLUME_15dB,       // 15dB
    INPUT_DIGITAL_VOLUME_30dB,       // 30dB
#else
    0x00
#endif
};


audioCodecCfg_t audioCodec_cfg;

static uint8_t is_codec_opened = FALSE;


/******************************************************************************
*                          Function prototype
******************************************************************************/
extern void utilslib_delayUs(UINT32 delay);


/******************************************************************************
*                          Function definitions
******************************************************************************/

/* AudioCodec_init */
void AudioCodec_init(void)
{
}

/*********************************************************************
 * @brief   codec open
 *
 * @param   none
 *
 * @return  status -    WICED_SUCCESS: if succeeded
 *                      otherwise : if failed to open
 */
wm8960_result_t AudioCodec_open(void)
{
    
    if (is_codec_opened)
    {
        return WM8960_SUCCESS;
    }

    AudioCodec_powerOn(TRUE);
   
    // initialize audio codec config

    audioCodec_cfg.bitsPerSample = AudioCodec_16_BIT;
    audioCodec_cfg.sampleRate = AUDIO_CODEC_SAMPLE_FREQ_8K;
    audioCodec_cfg.noOfChannels = CHANNEL_MONO; //AudioCodec_MONO;
    audioCodec_cfg.speaker_path = OUTPUT_PATH_DEFAULT;
    audioCodec_cfg.mic_path = INPUT_PATH_DEFAULT;
    audioCodec_cfg.interface_mode = IF_MODE_SLAVE; //codec is slave
    audioCodec_cfg.interface_format = IF_FORMAT_I2S;
    audioCodec_cfg.clk_mode = CLOCK_MODE_PLL_ON; //AudioCodec_CLOCK_PLL_ON;
    
#ifdef CODEC_WM8960
    audioCodec_cfg.enable_mclk = TRUE;
#else
    audioCodec_cfg.enable_mclk = FALSE;
#endif
    audioCodec_cfg.mic_amp_gain_level = MIC_AMP_GAIN_LEVEL_DEFAULT;
    audioCodec_cfg.mic_digital_volume_level = MIC_DIGITAL_VOLUME_LEVEL_DEFAULT;

    audioCodec_cfg.spk_amp_gain_level = SPK_AMP_GAIN_LEVEL_DEFAULT;
    audioCodec_cfg.spk_digital_volume_level = SPK_DIGITAL_VOLUME_LEVEL_DEFAULT;

    AudioCodec_reset();
    
#if defined(CODEC_WM8960)
    AudioCodec_config_new_wm8960();
#endif

    AudioCodec_micAmpGainCtrl(MIC_AMP_GAIN_LEVEL_DEFAULT);              // 25.5dB
#ifdef CODEC_WM8960 
    AudioCodec_micDigitalVolCtrl(MIC_DIGITAL_VOLUME_LEVEL_DEFAULT);     // 0dB
    AudioCodec_spkAmpGainCtrl(SPK_AMP_GAIN_LEVEL_DEFAULT);              // 0dB
#endif  
    AudioCodec_spkDigitalVolCtrl(SPK_DIGITAL_VOLUME_LEVEL_DEFAULT);     // 0dB

    is_codec_opened = TRUE;

    return WM8960_SUCCESS;
}

void AudioCodec_powerOn(uint8_t on)
{
    //board dependency
}

/*********************************************************************
 * @brief   Perform a soft reset of the device
 *
 * @param   none
 *
 * @return  none
 */
wm8960_result_t AudioCodec_reset(void)
{
    return WM8960_SUCCESS;
}

#ifdef CODEC_WM8960
wm8960_result_t AudioCodec_config_new_wm8960(void)
{
    reg_val_t Reg;
    
    /* ====================================================== */
    /* Power on */
    /* ====================================================== */

    //VIMDSEL = 0x03, VREF Power up, AINL = 1, AINR = 1
    Reg = AudioCodec_read_reg(pwr_mgmt1_addr);
    Reg &= ~(PWR_MGMT1_VREF_MASK | PWR_MGMT1_AINL_MASK | PWR_MGMT1_AINR_MASK);
    Reg |= (PWR_MGMT1_VMIDSEL << PWR_MGMT1_VMIDSEL_OFFSET) | (PWR_MGMT1_VREF << PWR_MGMT1_VREF_OFFSET) | (PWR_MGMT1_AINL << PWR_MGMT1_AINL_OFFSET) | (PWR_MGMT1_AINR << PWR_MGMT1_AINR_OFFSET);
    AudioCodec_write_reg(pwr_mgmt1_addr, Reg);


    /* ====================================================== */
    /* ADC Enable, MIC bias enable */
    /* ====================================================== */

    /*
        ADCL(Enable ADC left channel 0 = ADC disabled, 1 = ADC enabled)
    */

    //ADCL = 1, ADCR = 1, MICB = 1
    Reg = AudioCodec_read_reg(pwr_mgmt1_addr);
    Reg &= ~(PWR_MGMT1_ADCL_MASK | PWR_MGMT1_ADCR_MASK | PWR_MGMT1_MICB_MASK);
    Reg |= ((PWR_MGMT1_ADCL << PWR_MGMT1_ADCL_OFFSET) | (PWR_MGMT1_ADCR << PWR_MGMT1_ADCR_OFFSET) | (PWR_MGMT1_MICB << PWR_MGMT1_MICB_OFFSET));
    AudioCodec_write_reg(pwr_mgmt1_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* PLL Enable, DAC Enable */
    /* ====================================================== */

    /*
        PLL Enable
        DACL(Left channel DAC enable, 0 = DAC disabled, 1 = DAC enabled)
        DACR(Right channel DAC enable, 0 = DAC disabled, 1 = DAC enabled)
    */

    //PLL Enable
    Reg = AudioCodec_read_reg(pwr_mgmt2_addr);
    Reg &= ~(PWR_MGMT2_PLL_EN_MASK);
    Reg |= (PWR_MGMT2_PLL_EN << PWR_MGMT2_PLL_EN_OFFSET);
    AudioCodec_write_reg(pwr_mgmt2_addr, Reg);

    //DACL = 1, DACR = 1 DAC left right channel enable
    Reg = AudioCodec_read_reg(pwr_mgmt2_addr);
    Reg &= ~(PWR_MGMT2_DACL_MASK | PWR_MGMT2_DACR_MASK);
    Reg |= ((PWR_MGMT2_DACL << PWR_MGMT2_DACL_OFFSET) | (PWR_MGMT2_DACR << PWR_MGMT2_DACR_OFFSET));
    AudioCodec_write_reg(pwr_mgmt2_addr, Reg);

    /* ------------------------------------------------------ */

    /* ====================================================== */
    /* PLL Control  */
    /* MCLK = 12MHz, PLL = 12.288MHz, PLL Enable */
    /* ====================================================== */

    //PLL_N_SDM = 1, PLLPRESCALE = 1
    Reg = AudioCodec_read_reg(pll_n_addr);
    Reg &= ~(PLL_N_SDM_MASK | PLL_N_PLLRESCALE_MASK | PLL_N_PLLN_MASK);
    Reg |= ((PLL_N_SDM << PLL_N_SDM_OFFSET) | ((0x08 & PLL_N_PLLN) << PLL_N_PLLN_OFFSET));
    AudioCodec_write_reg(pll_n_addr, Reg);

    //SYSCLKDIV = 2, CLKSEL = 1(0 = SYSCLK derived from MCLK, 1 = SYSCLK derived PLL output)
    Reg = AudioCodec_read_reg(clocking1_addr);
    Reg &= ~(CLOCKING1_SYSCLKDIV_MASK | CLOCKING1_CLKSEL_MASK);
    Reg |= (((DIVIDE_SYSCLK_BY2 & CLOCKING1_SYSCLKDIV) << CLOCKING1_SYSCLKDIV_OFFSET) | (CLOCKING1_CLKSEL << CLOCKING1_CLKSEL_OFFSET));
    AudioCodec_write_reg(clocking1_addr, Reg);

    /* ------------------------------------------------------ */
    

    /* ====================================================== */
    /* ENABLING THE OUTPUTS */ 
    /* ====================================================== */

    /*
        LOUT1(LOUT1 Output Enable)
        ROUT1(ROUT1 Output Enable)
    */

    Reg = AudioCodec_read_reg(pwr_mgmt2_addr);
    Reg &= ~(PWR_MGMT2_LOUT1_MASK | PWR_MGMT2_ROUT1_MASK);
    Reg |= (PWR_MGMT2_LOUT1 << PWR_MGMT2_LOUT1_OFFSET) | (PWR_MGMT2_ROUT1 << PWR_MGMT2_ROUT1_OFFSET);
    AudioCodec_write_reg(pwr_mgmt2_addr, Reg);

    /* ------------------------------------------------------ */

    //LMIC = 1, RMIC = 1
    Reg = AudioCodec_read_reg(pwr_mgmt3_addr);
    Reg &= ~(PWR_MGMT3_LMIC_MASK | PWR_MGMT3_RMIC_MASK);
    Reg |= ((PWR_MGMT3_LMIC << PWR_MGMT3_LMIC_OFFSET) | (PWR_MGMT3_RMIC << PWR_MGMT3_RMIC_OFFSET));
    AudioCodec_write_reg(pwr_mgmt3_addr, Reg);
    

    /* ====================================================== */
    /* Output Mixer enable */
    /* ====================================================== */

    /*
        LOMIX(Left Output Mixer Enable Control, 0 = Disable, 1 = Enable)
        ROMIX(Right Output Mixer Enable Control, 0 = Disable, 1 = Enable)
    */

    //LOMIX = 1, ROMIX = 1 Enable
    Reg = AudioCodec_read_reg(pwr_mgmt3_addr);
    Reg &= ~(PWR_MGMT3_LOMIX_MASK | PWR_MGMT3_ROMIX_MASK);
    Reg |= ((PWR_MGMT3_LOMIX << PWR_MGMT3_LOMIX_OFFSET) | (PWR_MGMT3_ROMIX << PWR_MGMT3_ROMIX_OFFSET));
    AudioCodec_write_reg(pwr_mgmt3_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* Slave Mode with ADCLRC as GPIO & Jack detect mode */
    /* ====================================================== */

    /*
        ALRCGPIO/GPIO1
        GPIOSEL
    */

    //ALRCGPIO = 1 (use GPIO pin)
    Reg = AudioCodec_read_reg(audio_interface2_addr);
    Reg &= ~(AUDIO_INTERFACE2_ALRCGPIO_MASK);
    Reg |= AUDIO_INTERFACE2_ALRCGPIO << AUDIO_INTERFACE2_ALRCGPIO_OFFSET;
    AudioCodec_write_reg(audio_interface2_addr, Reg);

    //GPIOSEL
    Reg = AudioCodec_read_reg(additional_control4_addr);
    Reg &= ~(ADDITIONAL_CONTROL4_GPIOSEL_MASK);
    Reg |= (0x01 & ADDITIONAL_CONTROL4_GPIOSEL) << ADDITIONAL_CONTROL4_GPIOSEL_OFFSET;
    AudioCodec_write_reg(additional_control4_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* ADC, DAC Samping rate */
    /* 8K Samlping */
    /* Audio data word length 16bits*/
    /* Slave mode */
    /* ====================================================== */


    //ADCDIV = 110 ADC 8K Sampling
    Reg = AudioCodec_read_reg(clocking1_addr);
    Reg &= ~(CLOCKING1_ADCDIV_MASK);
    Reg |= ((0x06 & CLOCKING1_ADCDIV) << CLOCKING1_ADCDIV_OFFSET);
    AudioCodec_write_reg(clocking1_addr, Reg);

    //DACDIV = 110 DAC 8K Sampling
    Reg = AudioCodec_read_reg(clocking1_addr);
    Reg &= ~(CLOCKING1_DACDIV_MASK);
    Reg |= ((0x06 & CLOCKING1_DACDIV)<< CLOCKING1_DACDIV_OFFSET);
    AudioCodec_write_reg(clocking1_addr, Reg);

    //WL[1:0] (00 = 16bits, 01 = 20bits, 10 = 24bits, 11 = 32bits), MS = 0(Slave Mode)
    Reg = AudioCodec_read_reg(audio_interface1_addr);
    Reg &= ~(AUDIO_INTERFACE1_WL_MASK | AUDIO_INTERFACE1_MS_MASK);
    Reg |= ((AUDIO_WORD_LENGTH_16bit & AUDIO_INTERFACE1_WL) << AUDIO_INTERFACE1_WL_OFFSET) | ((AUDIO_CODEC_SLAVE & AUDIO_INTERFACE1_MS) << AUDIO_INTERFACE1_MS_OFFSET);
    AudioCodec_write_reg(audio_interface1_addr, Reg);
    
    /* ------------------------------------------------------ */

    //ALCSEL = 00(Auto Level Control function off)
    Reg = AudioCodec_read_reg(alc1_addr);
    Reg &= ~(ALC1_ALCSEL_MASK);
    AudioCodec_write_reg(alc1_addr, 0x0000);


    /* ====================================================== */
    /* INPUT configuration. */
    // Pseudo-differential MIC configuration
    /* ====================================================== */

    /*
        <Pseudo-differential MIC configration on left channel>

        AINL = 1    Power up(Analogue input PGA and Boost left) Note: LMIC must also be set to enable the PGA.
        LMIC = 1    Power up(Left Input PGA Enable) Note: PGA also requires AINL to be set.
        LINMUTE = 0 Disable mute(Left Input PGA Analogue Mute) Note: IPVU must be set to un-mute.
        LMN1 = 1    LINPUT1 connected to PGA(Connect LINPUT1 to inverting input of Left Input PGA)
        LMP2 = 1    LINPUT2 connected to PGA(Constant input impedance) Connect LINPUT2 to non-inverting input of Left Input PGA
        LMP3 = 0    LINPUT3 not connected to PGA(Connect LINPUT1 to inverting input of Left input PGA)
        LIN2BOOST = 000 Mute(3dB steps up to) LINPUT2 to Boost Mixer Gain
        LIN3BOOST = 000 Mute(3dB steps up to) LINPUT3 to Boost Mixer Gain
        LI2LO = 0   LINPUT3 to Left Output Mixer
        LMIC2B = 1  Connect Left Input PGA to Left Input Boostmixer
    */

    //LINMUTE = 0
    Reg = AudioCodec_read_reg(left_input_volume_addr);
    Reg &= ~(LEFT_INPUT_VOLUME_LINMUTE_MASK);
    AudioCodec_write_reg(left_input_volume_addr, Reg);

    //LMN1 = 1, LMP2 = 1, LMP3 = 0
    Reg = AudioCodec_read_reg(adcl_signal_path_addr);
    Reg &= ~(ADCL_SIGNAL_PATH_LMN1_MASK | ADCL_SIGNAL_PATH_LMP3_MASK | ADCL_SIGNAL_PATH_LMP2_MASK);
    Reg |= (ADCL_SIGNAL_PATH_LMN1 << ADCL_SIGNAL_PATH_LMN1_OFFSET) | (ADCL_SIGNAL_PATH_LMP2 << ADCL_SIGNAL_PATH_LMP2_OFFSET);
    AudioCodec_write_reg(adcl_signal_path_addr, Reg);

    //LIN2BOOST = 000, LIN3BOOST = 000
    Reg = AudioCodec_read_reg(input_boost_mixer1_addr);
    Reg &= ~(INPUT_BOOST_MIXER1_LIN3BOOST_MASK | INPUT_BOOST_MIXER1_LIN2BOOST_MASK);
    Reg |= ((IN_BOOST_MUTE & INPUT_BOOST_MIXER1_LIN3BOOST) << INPUT_BOOST_MIXER1_LIN3BOOST_OFFSET) | ((IN_BOOST_MUTE & INPUT_BOOST_MIXER1_LIN2BOOST) << INPUT_BOOST_MIXER1_LIN2BOOST_OFFSET) ;
    AudioCodec_write_reg(input_boost_mixer1_addr, Reg);

    //LI2LO = 0, LI2LOVOL = 000
    Reg = AudioCodec_read_reg(left_out_mix1_addr);
    Reg &= ~(LEFT_OUT_MIX1_LD2LO_MASK | LEFT_OUT_MIX1_LI2LOVOL_MASK);
    AudioCodec_write_reg(left_out_mix1_addr, Reg);

    //LMIC2B = 1
    Reg = AudioCodec_read_reg(adcl_signal_path_addr);
    Reg &= ~(ADCL_SIGNAL_PATH_LMIC2B_MASK);
    //Reg |= ((ADCL_SIGNAL_PATH_LMIC2B << ADCL_SIGNAL_PATH_LMIC2B_OFFSET) | ((0x01 & ADCL_SIGNAL_PATH_LMICBOOST) << ADCL_SIGNAL_PATH_LMICBOOST_OFFSET));
    Reg |= ((ADCL_SIGNAL_PATH_LMIC2B << ADCL_SIGNAL_PATH_LMIC2B_OFFSET));
    AudioCodec_write_reg(adcl_signal_path_addr, Reg);


    /* ------------------------------------------------------ */

    //RINMUTE = 0
    Reg = AudioCodec_read_reg(right_input_volume_addr);
    Reg &= ~(RIGHT_INPUT_VOLUME_RINMUTE_MASK);
    AudioCodec_write_reg(right_input_volume_addr, Reg);

    //RMN1 = 1, RMP2 = 1, RMP3 = 0
    Reg = AudioCodec_read_reg(adcr_signal_path_addr);
    Reg &= ~(ADCR_SIGNAL_PATH_RMN1_MASK | ADCR_SIGNAL_PATH_RMP2_MASK | ADCR_SIGNAL_PATH_RMP3_MASK);
    Reg |= (ADCR_SIGNAL_PATH_RMN1 << ADCR_SIGNAL_PATH_RMN1_OFFSET) | (ADCR_SIGNAL_PATH_RMP2 << ADCR_SIGNAL_PATH_RMP2_OFFSET);
    AudioCodec_write_reg(adcr_signal_path_addr, Reg);

    //RIN2BOOST = 000, RIN3BOOST = 000
    Reg = AudioCodec_read_reg(input_boost_mixer2_addr);
    Reg &= ~(INPUT_BOOST_MIXER2_RIN3BOOST_MASK | INPUT_BOOST_MIXER2_RIN2BOOST_MASK);
    Reg |= ((IN_BOOST_MUTE & INPUT_BOOST_MIXER2_RIN3BOOST) << INPUT_BOOST_MIXER2_RIN3BOOST_OFFSET) | ((IN_BOOST_MUTE & INPUT_BOOST_MIXER2_RIN2BOOST) << INPUT_BOOST_MIXER2_RIN2BOOST_OFFSET) ;
    AudioCodec_write_reg(input_boost_mixer2_addr, Reg);

    //RI2RO = 0, RI2ROVOL = 000
    Reg = AudioCodec_read_reg(right_out_mix2_addr);
    Reg &= ~(RIGHT_OUT_MIX2_RI2RO_MASK | RIGHT_OUT_MIX2_RI2ROVOL_MASK);
    Reg |= (RIGHT_OUT_MIX2_RI2RO << RIGHT_OUT_MIX2_RI2RO_OFFSET);
    AudioCodec_write_reg(right_out_mix2_addr, Reg);

    //RMIC2B = 1
    Reg = AudioCodec_read_reg(adcr_signal_path_addr);
    Reg &= ~(ADCR_SIGNAL_PATH_RMIC2B_MASK);
    Reg |= ADCR_SIGNAL_PATH_RMIC2B << ADCR_SIGNAL_PATH_RMIC2B_OFFSET;
    AudioCodec_write_reg(adcr_signal_path_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* DAC Mono Mix */
    /* ====================================================== */

    /*
            DATSEL bits are used to select which channel is used for the left and right ADC data.
        {
            00: LEFT-RIGHT(Common)
            01: LEFT-LEFT
            10: RIGHT-RIGHT
            11: RIGHT-LEFT
        }
    */

    // DMONOMIX (0 = Stereo, 1 = Mono), DATSEL[1:0] = 00
    Reg = AudioCodec_read_reg(additional_control1_addr);
    Reg &= ~(ADDITIONAL_CONTROL1_DMONOMIX_MASK);
    Reg |= (ADDITIONAL_CONTROL1_DMONOMIX << ADDITIONAL_CONTROL1_DMONOMIX_OFFSET) | ((LEFT_LEFT & ADDITIONAL_CONTROL1_DATSEL) << ADDITIONAL_CONTROL1_DATSEL_OFFSET);
    //Reg |= ((LEFT_RIGHT & ADDITIONAL_CONTROL1_DATSEL) << ADDITIONAL_CONTROL1_DATSEL_OFFSET);
    AudioCodec_write_reg(additional_control1_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* Thermal shutdown */
    /* ====================================================== */

    /*
        TSENSEN must be set to 1 to enable the temperature sensor when using the TSDEN thermal shutdown function.
    */

    // TSENSEN = 0(Temperature Sensor Disable)
    Reg = AudioCodec_read_reg(additional_control4_addr);
    Reg &= ~(ADDITIONAL_CONTROL4_TSENSEN_MASK);
    AudioCodec_write_reg(additional_control4_addr, Reg);

    // TSDEN = 0(Thermal Shutdown Disable)
    Reg = AudioCodec_read_reg(additional_control1_addr);
    Reg &= ~(ADDITIONAL_CONTROL1_TSDEN_MASK);
    //Reg |= (THERMAL_SHUTDOWN_ENABLE & ADDITIONAL_CONTROL1_TSDEN) << ADDITIONAL_CONTROL1_TSDEN_OFFSET;
    AudioCodec_write_reg(additional_control1_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* Output Mixer path */
    /* ====================================================== */

    /*
        LD2LO(Left DAC to Left Output Mixer, 0 = Disable(Mute), 1 = Enable Path)
        RD2RO(Right DAC to Right Output Mixer, 0 = Disable(Mute), 1 = Enable Path)
    */

    //LD2LO = 1 Enable Path
    Reg = AudioCodec_read_reg(left_out_mix1_addr);
    Reg &= ~(LEFT_OUT_MIX1_LD2LO_MASK);
    Reg |= LEFT_OUT_MIX1_LD2LO << LEFT_OUT_MIX1_LD2LO_OFFSET;
    AudioCodec_write_reg(left_out_mix1_addr, Reg);

    //RD2RO = 1 Enable Path
    Reg = AudioCodec_read_reg(right_out_mix2_addr);
    Reg &= ~(RIGHT_OUT_MIX2_RD2RO_MASK);
    Reg |= RIGHT_OUT_MIX2_RD2RO << RIGHT_OUT_MIX2_RD2RO_OFFSET;
    AudioCodec_write_reg(right_out_mix2_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* DAC Soft mute */
    /* ====================================================== */

    /*
        DACMU disable
    */

    //DACMU = 0, No mute
    Reg = AudioCodec_read_reg(adc_dac_control1_addr);
    Reg &= ~(ADC_DAC_CONTROL1_DACMU_MASK);
    AudioCodec_write_reg(adc_dac_control1_addr, Reg);

    /* ------------------------------------------------------ */


    /* ====================================================== */
    /* HP_L AND HP_R OUTPUTS */
    /* ====================================================== */

    /*
        OUT1VU(Headphone Volume Update, 0 = Store LOUT1VOL in intermediate latch(no gain change), 1 = update left and right channel gains)
        LOUT1VOL(LOUT1 Volume)
    */

    //OUT1VU set to 0(left)
    Reg = AudioCodec_read_reg(lout1_volume_addr);
    Reg &= ~(LOUT1_VOLUME_OUT1VU_MASK);
    AudioCodec_write_reg(lout1_volume_addr, Reg);

    //Update LOUT1VOL
    Reg = AudioCodec_read_reg(lout1_volume_addr);
    Reg &= ~(LOUT1_VOLUME_LOUT1VOL_MASK);
    Reg |= ((OUTPUT_AMP_GAIN_0db & LOUT1_VOLUME_LOUT1VOL) << LOUT1_VOLUME_LOUT1VOL_OFFSET);
    AudioCodec_write_reg(lout1_volume_addr, Reg);

    //OUT1VU set to 1(left)
    Reg = AudioCodec_read_reg(lout1_volume_addr);
    Reg &= ~(LOUT1_VOLUME_OUT1VU_MASK);
    Reg |= LOUT1_VOLUME_OUT1VU << LOUT1_VOLUME_OUT1VU_OFFSET;
    AudioCodec_write_reg(lout1_volume_addr, Reg);

    //OUT1VU set to 0(right)
    Reg = AudioCodec_read_reg(rout1_volume_addr);
    Reg &= ~(ROUT1_VOLUME_OUT1VU_MASK);
    AudioCodec_write_reg(rout1_volume_addr, Reg);

    //Update ROUT1VOL
    Reg = AudioCodec_read_reg(rout1_volume_addr);
    Reg &= ~(ROUT1_VOLUME_ROUT1VOL_MASK);
    Reg |= (OUTPUT_AMP_GAIN_0db & ROUT1_VOLUME_ROUT1VOL) << ROUT1_VOLUME_ROUT1VOL_OFFSET;
    AudioCodec_write_reg(rout1_volume_addr, Reg);

    //OUT1VU set to 1(right)
    Reg = AudioCodec_read_reg(rout1_volume_addr);
    Reg &= ~(ROUT1_VOLUME_OUT1VU_MASK);
    Reg |= ROUT1_VOLUME_OUT1VU << ROUT1_VOLUME_OUT1VU_OFFSET;
    AudioCodec_write_reg(rout1_volume_addr, Reg);

    /* ------------------------------------------------------ */

    return WM8960_SUCCESS;

}
#endif

/*
 * speaker amp volume up
 */
uint8_t AudioCodec_spkVol_up(void)
{
    uint8_t spkVol_level;

    if (!is_codec_opened)
        return 0xFF;

#ifdef CODEC_WM8960
    if (audioCodec_cfg.spk_amp_gain_level == SPK_AMP_GAIN_LEVEL_NUM)
    {
        return SPK_AMP_GAIN_LEVEL_NUM;
    }

    spkVol_level = audioCodec_cfg.spk_amp_gain_level + 1;
    AudioCodec_spkAmpGainCtrl(spkVol_level);
#else
    if (audioCodec_cfg.spk_digital_volume_level == SPK_DIGITAL_VOLUME_LEVEL_NUM)
    {
        return SPK_DIGITAL_VOLUME_LEVEL_NUM;
    }

    spkVol_level = audioCodec_cfg.spk_digital_volume_level + 1;
    AudioCodec_spkDigitalVolCtrl(spkVol_level);

#endif
    

    return spkVol_level;
}

/*
 * speaker amp volume down
 */
uint8_t AudioCodec_spkVol_down(void)
{
    uint8_t spkVol_level;

    if (!is_codec_opened)
        return 0xFF;
    
#ifdef CODEC_WM8960
    if( audioCodec_cfg.spk_amp_gain_level == SPK_AMP_GAIN_LEVEL0)
    {
        return SPK_AMP_GAIN_LEVEL0;
    }

    spkVol_level = audioCodec_cfg.spk_amp_gain_level - 1;
    
    AudioCodec_spkAmpGainCtrl(spkVol_level);
#else
    if( audioCodec_cfg.spk_digital_volume_level == SPK_DIGITAL_VOLUME_LEVEL0)
    {
        return SPK_DIGITAL_VOLUME_LEVEL0;
    }

    spkVol_level = audioCodec_cfg.spk_digital_volume_level - 1;

    AudioCodec_spkDigitalVolCtrl(spkVol_level);
#endif

    return spkVol_level;

}

/*
 * speaker digital volume up
 */
uint8_t AudioCodec_DAC_spkVol_up(void)
{
    uint8_t spkVol_level;

    if (!is_codec_opened)
        return 0xFF;

    if (audioCodec_cfg.spk_digital_volume_level == SPK_DIGITAL_VOLUME_LEVEL_NUM)
    {
        return SPK_DIGITAL_VOLUME_LEVEL_NUM;
    }

    spkVol_level =  audioCodec_cfg.spk_digital_volume_level + 1;
    AudioCodec_spkDigitalVolCtrl(spkVol_level);

    return spkVol_level;    
}

/*
 * speaker ditital volume down
 */
uint8_t AudioCodec_DAC_spkVol_down(void)
{
    uint8_t spkVol_level;

    if (!is_codec_opened)
        return 0xFF;

    if (audioCodec_cfg.spk_digital_volume_level == SPK_DIGITAL_VOLUME_LEVEL0)
    {
        return SPK_DIGITAL_VOLUME_LEVEL0;
    }

    spkVol_level =  audioCodec_cfg.spk_digital_volume_level - 1;
    AudioCodec_spkDigitalVolCtrl(spkVol_level);

    return spkVol_level;    
}

wm8960_result_t AudioCodec_spkAmpGainCtrl(uint8_t spkAmpGainLevel)
{
    reg_val_t gain_val;
    reg_val_t reg;

    gain_val = (uint8_t)spk_amp_gain_val_tbl[spkAmpGainLevel];
    if(spkAmpGainLevel == SPK_AMP_GAIN_LEVEL_NUM)
    {
        return WM8960_SUCCESS;
    }

#if defined(CODEC_WM8960)
    WM8960_Output_AmpVolumeControl(gain_val);
#endif  
    audioCodec_cfg.spk_amp_gain_level = spkAmpGainLevel;
    return WM8960_SUCCESS;
}

wm8960_result_t AudioCodec_spkDigitalVolCtrl(uint8_t volumeLevel)
{
    reg_val_t volume_val;
    reg_val_t reg;

    if(volumeLevel == SPK_DIGITAL_VOLUME_LEVEL_NUM)
    {
        return WM8960_SUCCESS;
    }
    volume_val = spk_digital_volume_val_tbl[volumeLevel];

#if defined(CODEC_WM8960)
        WM8960_Output_DigitalVolumeControl(volume_val);
#endif
    audioCodec_cfg.spk_digital_volume_level = volumeLevel;
    return WM8960_SUCCESS;
}


/*********************************************************************
 * @brief   Mute Audio line out
 *
  * @param[in] speaker        - Audio out id. (headphone, line out, all etc..)
 *                               Please refer Audio out Macro section
 *
 * @return WICED_SUCCESS on success else -ve.
 */

wm8960_result_t AudioCodec_speakerMute(void)
{
#if defined(CODEC_WM8960)
    WM8960_Output_AmpVolumeControl(OUTPUT_AMP_GAIN_MUTE);
#endif
    return WM8960_SUCCESS;
}

/*********************************************************************
 * @brief   Unmute audio line out
 *
  * @param[in] speaker        - Audio out id. (headphone, line out, all etc..)
 *                               Please refer Audio out Macro section
 *
 * @return 0 on success else -ve.
 */

wm8960_result_t AudioCodec_speakerUnmute(void)
{
    uint8_t spkVol_level;

    spkVol_level =  audioCodec_cfg.spk_digital_volume_level;

#if defined(CODEC_WM8960)
    WM8960_Output_DigitalVolumeControl(spkVol_level);
#endif
    return WM8960_SUCCESS;
}

/*
 * mic volume up
 */
uint8_t AudioCodec_micVol_up(void)
{
    uint8_t micVol_level;

    if (!is_codec_opened)
        return 0xFF;

    if (audioCodec_cfg.mic_amp_gain_level == (MIC_AMP_GAIN_LEVEL_NUM - 1))
    {
        return (MIC_AMP_GAIN_LEVEL_NUM - 1);
    }

    micVol_level =  audioCodec_cfg.mic_amp_gain_level + 1;

    AudioCodec_micAmpGainCtrl(micVol_level);

    return micVol_level;    
}

/*
 * mic volume down
 */
uint8_t AudioCodec_micVol_down(void)
{
    uint8_t micVol_level;

    if (!is_codec_opened)
        return 0xFF;
    
    if (audioCodec_cfg.mic_amp_gain_level == MIC_AMP_GAIN_LEVEL0)
    {
        return MIC_AMP_GAIN_LEVEL0;
    }

    micVol_level = audioCodec_cfg.mic_amp_gain_level - 1;

    AudioCodec_micAmpGainCtrl(micVol_level);

    return micVol_level;    
}

uint8_t AudioCodec_ADC_micVol_up(void)
{
    uint8_t micVol_level;

    if (!is_codec_opened)
        return 0xFF;

    if (audioCodec_cfg.mic_digital_volume_level == (MIC_DIGITAL_VOLUME_LEVEL_NUM - 1))
    {
        return (MIC_DIGITAL_VOLUME_LEVEL_NUM - 1);
    }

    micVol_level = audioCodec_cfg.mic_digital_volume_level + 1;

    AudioCodec_micDigitalVolCtrl(micVol_level);

    return micVol_level;    
}

/*
 * mic volume down
 */
uint8_t AudioCodec_ADC_micVol_down(void)
{
    uint8_t micVol_level;

    if (!is_codec_opened)
        return 0xFF;

    if (audioCodec_cfg.mic_digital_volume_level == MIC_DIGITAL_VOLUME_LEVEL0)
    {
        return MIC_DIGITAL_VOLUME_LEVEL0;
    }

    micVol_level =  audioCodec_cfg.mic_digital_volume_level - 1;

    AudioCodec_micDigitalVolCtrl(micVol_level);

    return micVol_level;    
}

// mic amp gain
wm8960_result_t AudioCodec_micAmpGainCtrl(uint8_t gain_level)
{
    reg_val_t gain_val;

    gain_val = mic_amp_gain_val_tbl[gain_level];
    if (gain_level == (MIC_AMP_GAIN_LEVEL_NUM - 1))
    {
        return WM8960_SUCCESS;
    }

#if defined(CODEC_WM8960)
    WM8960_Mic_AmpVolumeControl(gain_val);
#endif
    audioCodec_cfg.mic_amp_gain_level = gain_level;
    return WM8960_SUCCESS;
}

//mic Digital volume control
wm8960_result_t AudioCodec_micDigitalVolCtrl(uint8_t volumeLevel)
{
    reg_val_t mic_volume_val;

    mic_volume_val = mic_digital_gain_val_tbl[volumeLevel];
    if (volumeLevel == (MIC_DIGITAL_VOLUME_LEVEL_NUM - 1))
    {
        return WM8960_SUCCESS;
    }
#if defined(CODEC_WM8960)
    WM8960_Mic_DigitalVolumeControl(mic_volume_val);
#endif
    audioCodec_cfg.mic_digital_volume_level = volumeLevel;
    return WM8960_SUCCESS;
}

/*********************************************************************
 * @brief   Mute Audio line in
 *
  * @param[in] mic           - Audio in id. (line in, mono mic, all etc..)
 *                          Please refer Audio In Macro section
 *
 * @return 0 on success else -ve.
 */
wm8960_result_t AudioCodec_micMute(void)
{
    uint8_t regVal;
    
    if (!is_codec_opened)
        return 0xFF;
  

#if defined(CODEC_WM8960)
    WM8960_Mic_MuteControl(TRUE);
#endif
    return WM8960_SUCCESS;
}

/*********************************************************************
 * @brief   Unmute audio line
 *
  * @param[in] mic       - Audio in id. (line in, mono mic, all etc..)
 *                      Please refer Audio In Macro section
 *
 * @return 0 on success else -ve.
 */
wm8960_result_t AudioCodec_micUnmute(void)
{
    uint8_t regVal;
    uint8_t mic_digital_vol;
    
    if (!is_codec_opened)
        return 0xFF;

    mic_digital_vol =  mic_digital_gain_val_tbl[audioCodec_cfg.mic_digital_volume_level];

#if defined(CODEC_WM8960)
    WM8960_Mic_MuteControl(FALSE);
#endif
    return WM8960_SUCCESS;
}

/* set speaker mode : mono or stereo */
wm8960_result_t AudioCodec_setSpkMode(uint8_t mode)
{
    unsigned char regVal;

    switch (mode)
    {
        case CHANNEL_MONO:
#if defined(CODEC_WM8960)
            WM8960_SetOutput_Config(AUDIO_OUTPUT_MONO);
#endif
            break;

        case CHANNEL_STEREO:
#if defined(WM8960)
            WM8960_SetOutput_Config(AUDIO_OUTPUT_STEREO);
#endif          
            break;

        default:
            return WM8960_ERROR;
            break;
    }
    return WM8960_SUCCESS;
}

wm8960_result_t AudioCodec_setSampleFreq(sample_freq_t freq)
{
    uint8_t fs;
    
    switch (freq)
    {
        case AUDIO_CODEC_SAMPLE_FREQ_8K:
#if defined(CODEC_WM8960)           
            fs = SAMPLING_RATE_8K;
#endif 
            break;

        case AUDIO_CODEC_SAMPLE_FREQ_16K:
#if defined(CODEC_WM8960)           
            fs = SAMPLING_RATE_16K;
#endif
            break;

        case AUDIO_CODEC_SAMPLE_FREQ_32K:
#if defined(CODEC_WM8960)           
            fs = SAMPLING_RATE_32K;
#endif
            break;

        case AUDIO_CODEC_SAMPLE_FREQ_44K:
#if defined(CODEC_WM8960)           
            fs = SAMPLING_RATE_44K;
#endif
            break;

        case AUDIO_CODEC_SAMPLE_FREQ_48K:
#if defined(CODEC_WM8960)           
            fs = SAMPLING_RATE_48K;
#endif
            break;

        default:
            return WM8960_ERROR;
            break;
    }

#if defined(CODEC_WM8960)
    WM8960_set_sample_freq(fs);
#endif
  
    return WM8960_SUCCESS;
}



