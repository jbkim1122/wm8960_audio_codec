/*******************************************************************************
 * Brief:   Library for WM8960, run on 20719
 * Author:  Jinbae
 * Update:  2020.11.12
 *******************************************************************************/

#include "data_types.h"
#include "audioCodec.h"
#include "wm8960.h"

#define WM8960_REG_NUM    0x38

/* 
 * wm8960 register cache 
 * We can't read the WM8960 register space when we are 
 * using 2 wire for device control, so we cache them instead. 
 */  
static uint16_t wm8960_reg_value[WM8960_REG_NUM] = {  
    0x0097, // 00H - Left Input volume
	0x0097, // 01H - Right Input volume
	0x0000, 
	0x0000,  
    0x0000, 
    0x0008, 
    0x0000, 
    0x000a,  
    0x01c0, 
    0x0000,
    0x00ff,
    0x00ff,  
    0x0000,
    0x0000,
    0x0000,
    0x0000, /* r15 */  
    0x0000,
    0x007b, 
    0x0100,
    0x0032,  
    0x0000, 
    0x00c3,
    0x00c3,
    0x01c0,  
    0x0000,
    0x0000, 
    0x0000, 
    0x0000,  
    0x0000,
    0x0000,
    0x0000,
    0x0000, /* r31 */  
    0x0100,
    0x0100,
    0x0050,
    0x0050,  
    0x0050,
    0x0050, 
    0x0000, 
    0x0000,  
    0x0000, 
    0x0000,
    0x0040, 
    0x0000,  
    0x0000, 
    0x0050,
    0x0050,
    0x0000, /* 47 */  
    0x0002, 
    0x0037,
    0x004d,
    0x0080,  
    0x0008,
    0x0031,
    0x0026, 
    0x00e9 
};  

sample_rate_t wm8960_sample_rate_val[SAMPLING_RATE_NUM] = 
{
    0x06, // 8k
    0x04, // 12k
    0x03, // 16k
    0x02, // 24k
    0x01, // 32k
    0x00, // 44.1k
    0x00  // 48k
};

sidetone_val_t wm8960_sidetone_val[LEFT_AUDIO_BYPASS_GAIN_MAX] =
{
	LEFT_AUDIO_BYPASS_GAIN_MINUS_21db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_18db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_15db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_12db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_9db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_6db,
	LEFT_AUDIO_BYPASS_GAIN_MINUS_3db,
	LEFT_AUDIO_BYPASS_GAIN_0db
};

/******************************************************************************
*                       Variables
******************************************************************************/


void WM8960_Aux_control(uint8_t toggle_val)
{
	reg_val_t regVal;

	if(toggle_val)
	{
		regVal = WM8960_Read_Reg(input_boost_mixer1_addr);
		regVal &= ~(INPUT_BOOST_MIXER1_LIN3BOOST_MASK);
		regVal |= ((IN_BOOST_0db & INPUT_BOOST_MIXER1_LIN3BOOST) << INPUT_BOOST_MIXER1_LIN3BOOST_OFFSET);
		WM8960_Write_Reg(input_boost_mixer1_addr, regVal);
		
	}
	else
	{
		regVal = WM8960_Read_Reg(input_boost_mixer1_addr);
		regVal &= ~(INPUT_BOOST_MIXER1_LIN3BOOST_MASK);
		WM8960_Write_Reg(input_boost_mixer1_addr, regVal);	
	}
}

void WM8960_Mic_MuteControl(uint8_t toggle_val)
{
	reg_val_t regVal;

	if(toggle_val)
	{
		regVal = AudioCodec_read_reg(pwr_mgmt1_addr);
		regVal &= ~(PWR_MGMT1_ADCL_MASK | PWR_MGMT1_ADCR_MASK);
		AudioCodec_write_reg(pwr_mgmt1_addr, regVal);
	}
	else
	{
		regVal = AudioCodec_read_reg(pwr_mgmt1_addr);
		regVal &= ~(PWR_MGMT1_ADCL_MASK | PWR_MGMT1_ADCR_MASK);
		regVal |= ((PWR_MGMT1_ADCL << PWR_MGMT1_ADCL_OFFSET));
		AudioCodec_write_reg(pwr_mgmt1_addr, regVal);
	}

}

void WM8960_Mic_AmpVolumeControl(Input_AMP_MGAIN_T mgain)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(left_input_volume_addr);
    regVal &= ~(LEFT_INPUT_VOLUME_LINVOL_MASK);
    regVal |= ((LEFT_INPUT_VOLUME_IPVU << LEFT_INPUT_VOLUME_IPVU_OFFSET) | ((mgain & LEFT_INPUT_VOLUME_LINVOL) << LEFT_INPUT_VOLUME_LINVOL_OFFSET));
	WM8960_Write_Reg(left_input_volume_addr, regVal);
	WM8960_Write_Reg(right_input_volume_addr, regVal);

}
void WM8960_Mic_DigitalVolumeControl(I_MIC_DVOL_T mic_volume_val)
{
	reg_val_t regVal;
	
	regVal = WM8960_Read_Reg(left_adc_volume_addr);
	regVal &= ~(LEFT_ADC_VOLUME_LADCVOL_MASK);
	regVal |= (((mic_volume_val & LEFT_ADC_VOLUME_LADCVOL) << LEFT_ADC_VOLUME_LADCVOL_OFFSET) | (LEFT_ADC_VOLUME_ADCVU << LEFT_ADC_VOLUME_ADCVU_OFFSET));
	WM8960_Write_Reg(left_adc_volume_addr, regVal);
	WM8960_Write_Reg(right_adc_volume_addr, regVal);
}

void WM8960_Output_DigitalVolumeControl(unsigned char DVOL)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(left_dac_volume_addr);
	regVal &= ~(LEFT_DAC_VOLUME_DACVU_MASK | LEFT_DAC_VOLUME_LDACVOL_MASK);
	regVal |= (((DVOL & LEFT_DAC_VOLUME_LDACVOL) << LEFT_DAC_VOLUME_LDACVOL_OFFSET) | (LEFT_DAC_VOLUME_DACVU << LEFT_DAC_VOLUME_DACVU_OFFSET));
	WM8960_Write_Reg(left_dac_volume_addr, regVal);
	WM8960_Write_Reg(right_dac_volume_addr, regVal);
}

void WM8960_Output_AmpVolumeControl(unsigned char AVOL)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(lout1_volume_addr);
	regVal &= ~(LOUT1_VOLUME_LOUT1VOL_MASK | LOUT1_VOLUME_OUT1VU_MASK);
	regVal |= (((AVOL & LOUT1_VOLUME_LOUT1VOL) << LOUT1_VOLUME_LOUT1VOL_OFFSET) | (LOUT1_VOLUME_OUT1VU << LOUT1_VOLUME_OUT1VU_OFFSET));
	WM8960_Write_Reg(lout1_volume_addr, regVal);

	regVal = WM8960_Read_Reg(rout1_volume_addr);
	regVal &= ~(ROUT1_VOLUME_ROUT1VOL_MASK | ROUT1_VOLUME_OUT1VU_MASK);
	regVal |= (((AVOL & ROUT1_VOLUME_ROUT1VOL) << ROUT1_VOLUME_ROUT1VOL_OFFSET) | (ROUT1_VOLUME_OUT1VU << ROUT1_VOLUME_OUT1VU_OFFSET));
	WM8960_Write_Reg(rout1_volume_addr, regVal);

}

//Stereo = 0, Mono = 1
void WM8960_SetOutput_Config(uint8_t val)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(additional_control1_addr);
	regVal &= ~(ADDITIONAL_CONTROL1_DMONOMIX_MASK);
	regVal |= (val & ADDITIONAL_CONTROL1_DMONOMIX) << ADDITIONAL_CONTROL1_DMONOMIX_OFFSET;
	WM8960_Write_Reg(additional_control1_addr, regVal);
}

void WM8960_set_sample_freq(uint8_t sample_freq)
{
    reg_val_t Reg;
    uint8_t pll_n_val;
	uint8_t pll_k1_val;
	uint8_t pll_k2_val;
	uint8_t pll_k3_val;
	uint8_t fs_val;

    uint8_t pll_clock = PLL_CLK_MODE_12_288M; // 12.288M

    if (sample_freq >= SAMPLING_RATE_NUM)
		return; // error

    fs_val = wm8960_sample_rate_val[sample_freq];  

    if (sample_freq == SAMPLING_RATE_44K)
    {
        pll_clock = PLL_CLK_MODE_11_289M;
   	}

    switch (pll_clock)
    {
        case PLL_CLK_MODE_12_288M:
			pll_n_val = 0x08;
		    pll_k1_val = 0x31;
		    pll_k2_val = 0x26;
		    pll_k3_val = 0xE9;	
			break;
			
        case PLL_CLK_MODE_11_289M:
            pll_n_val = 0x07;
		    pll_k1_val = 0x86;
		    pll_k2_val = 0xC2;
		    pll_k3_val = 0x26;			
			break;		

		default:
			break;
    }

	/* ====================================================== 
	 * PLL Control	
	 * at this moment, we support 12.288MHz or 11.2896MHz only.
	 * to support another clock you have to extend default_clock feature. 
	 * MCLK = 12MHz, PLL = 12.288MHz or 11.2896MHz, PLL Enable 	 
	/* ====================================================== */

	//PLL_N_SDM = 1, PLLPRESCALE = 1
	Reg = AudioCodec_read_reg(pll_n_addr);
	Reg &= ~(PLL_N_PLLN_MASK);
	Reg |= ((pll_n_val & PLL_N_PLLN) << PLL_N_PLLN_OFFSET);
	AudioCodec_write_reg(pll_n_addr, Reg);

	Reg = AudioCodec_read_reg(pll_k1_addr);
	Reg &= ~(PLL_K1_PLLK_MASK);
	Reg |= pll_k1_val;
	AudioCodec_write_reg(pll_k1_addr, Reg);
	
	Reg = AudioCodec_read_reg(pll_k2_addr);
	Reg &= ~(PLL_K2_PLLK_MASK);
	Reg |= pll_k2_val;
	AudioCodec_write_reg(pll_k2_addr, Reg);
	
	Reg = AudioCodec_read_reg(pll_k3_addr);
	Reg &= ~(PLL_K3_PLLK_MASK);
	Reg |= pll_k3_val;
	AudioCodec_write_reg(pll_k3_addr, Reg);

	//ADCDIV - ADC sample rate
	Reg = AudioCodec_read_reg(clocking1_addr);
	Reg &= ~(CLOCKING1_ADCDIV_MASK);
	Reg |= ((fs_val & CLOCKING1_ADCDIV) << CLOCKING1_ADCDIV_OFFSET);
	AudioCodec_write_reg(clocking1_addr, Reg);

	//DACDIV - DAC sample rate
	Reg = AudioCodec_read_reg(clocking1_addr);
	Reg &= ~(CLOCKING1_DACDIV_MASK);
	Reg |= ((fs_val & CLOCKING1_DACDIV)<< CLOCKING1_DACDIV_OFFSET);
	AudioCodec_write_reg(clocking1_addr, Reg);
}


void WM8960_MIC_BoostGain_ctrl(uint8_t val)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(adcl_signal_path_addr);
	regVal &= ~(ADCL_SIGNAL_PATH_LMICBOOST_MASK);
	regVal |= ((val & ADCL_SIGNAL_PATH_LMICBOOST) << ADCL_SIGNAL_PATH_LMICBOOST_OFFSET);
	WM8960_Write_Reg(adcl_signal_path_addr, regVal);
}

void WM8960_Sidetone_ctrl(uint8_t val)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(bypass1_addr);
	regVal &= ~(BYPASS1_LB2LO_MASK);
	if(val)
	{
		regVal |= (BYPASS1_LB2LO << BYPASS1_LB2LO_OFFSET);
	}
	WM8960_Write_Reg(bypass1_addr, regVal);
	WM8960_Write_Reg(bypass2_addr, regVal);
}

void WM8960_Sidetone_vol_ctrl(uint8_t val)
{
	reg_val_t regVal;

	regVal = WM8960_Read_Reg(bypass1_addr);
	regVal &= ~(BYPASS1_LB2LOVOL_MASK);

	if(((sidetone_level + val) == 0) || ((sidetone_level + val) == LEFT_AUDIO_BYPASS_GAIN_MAX))
	{
		return;
	}
	
	if(val)
	{
		sidetone_level++;
	}
	else
	{
		sidetone_level--;
	}
	
	regVal |= (wm8960_sidetone_val[sidetone_level] << BYPASS1_LB2LOVOL_OFFSET);
	WM8960_Write_Reg(bypass1_addr, regVal);
	WM8960_Write_Reg(bypass2_addr, regVal);
	
}

void WM8960_Write_Reg(uint8_t regAddr, uint16_t regVal)
{
    uint8_t writeDataBuf[2]; 
    
	writeDataBuf[0] = (((regAddr << 1) & 0xFE) | ((regVal >> 8) & 0x01));
	writeDataBuf[1] = (regVal & 0xFF);

    if (WM8960_SUCCESS == wiced_hal_i2c_write(writeDataBuf, 2, WM8960_Addr))
    {
        wm8960_reg_value[regAddr] = regVal;
    }
}

uint16_t WM8960_Read_Reg(uint8_t regAddr)
{
    return wm8960_reg_value[regAddr];
}

void WM8960_Reg_debug(void)
{
	extern uint16_t wm8960_reg_value[];
	uint8_t i;
	int8_t j;
	
	uint16_t reg_val;

	printf("==================== \n");
	printf("CODEC REG DEBUG \n"); 
	printf("==================== \n");
	
	for (i=0; i<0x38; i++)
	{
	    reg_val = wm8960_reg_value[i];
		printf("REG[%x] : ", i);
			
	    for (j=8; j>=0; j--)
	    {
	        if (reg_val & (0x01 << j))
	        {
			    printf("1");	
	        }
			else
			{
                printf("0");	 
			}

			if (!(j % 4))
				printf(" ");
	    }

		printf("\n");
	}
	printf("==================== \n");

}


