#ifndef AUDIOCODEC_H_
#define AUDIOCODEC_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "data_types.h"
#include "bt_types.h"

#include "../app_config.h"

#ifdef CODEC_WM8960
#include "wm8960.h"
#endif

/******************************************************************************
*                         Definitions
******************************************************************************/

#define TRUE    1
#define WM8960_SUCCESS 0
#define WM8960_ERROR   1001

// output path
#define OUTPUT_SPEAKER_OUT           0
#define OUTPUT_LINE_OUT              1
#define OUTPUT_HP_OUT                2

// input path
#define INPUT_MIC_IN                 0
#define INPUT_LINE_IN                1

// Bits per sample Macro
#define AudioCodec_8_BIT            8
#define AudioCodec_16_BIT           16
#define AudioCodec_20_BIT           20
#define AudioCodec_24_BIT           24

// Number of Channels Macro
#define CHANNEL_DEFAULT              0
#define CHANNEL_MONO                 1
#define CHANNEL_STEREO               2
#define CHANNEL_REVERSE_MONO         3
#define CHANNEL_REVERSE_STEREO       4


// interface mode - master or slave
#define IF_MODE_MASTER               0
#define IF_MODE_SLAVE                1

// interface format - pcm, i2s,///
#define IF_FORMAT_PCM                0
#define IF_FORMAT_I2S                1

// clock mode
#define CLOCK_MODE_PLL_OFF           0
#define CLOCK_MODE_PLL_ON            1

/******************************************************************************
*                         Type Defines
******************************************************************************/
typedef uint16_t wm8960_result_t;

enum
{
#if defined(CODEC_WM8960)
    SPK_AMP_GAIN_LEVEL0,    // mute
    SPK_AMP_GAIN_LEVEL1,    // -12dB
    SPK_AMP_GAIN_LEVEL2,    // -10dB
    SPK_AMP_GAIN_LEVEL3,    // -8dB
    SPK_AMP_GAIN_LEVEL4,    // -6dB
    SPK_AMP_GAIN_LEVEL5,    // -4dB
    SPK_AMP_GAIN_LEVEL6,    // -2dB
    SPK_AMP_GAIN_LEVEL7,    // 0dB
    SPK_AMP_GAIN_LEVEL8,    // 2dB
    SPK_AMP_GAIN_LEVEL9,    // 4dB  
#endif
    SPK_AMP_GAIN_LEVEL_NUM  // number of level
};
typedef uint8_t spk_amp_gain_level_t;

enum
{
#if defined(CODEC_WM8960)
    SPK_DIGITAL_VOLUME_LEVEL0,
    SPK_DIGITAL_VOLUME_LEVEL1,  
    SPK_DIGITAL_VOLUME_LEVEL2, 
    SPK_DIGITAL_VOLUME_LEVEL3, 
    SPK_DIGITAL_VOLUME_LEVEL4, 
    SPK_DIGITAL_VOLUME_LEVEL5, 
    SPK_DIGITAL_VOLUME_LEVEL6, 
    SPK_DIGITAL_VOLUME_LEVEL7, 
    SPK_DIGITAL_VOLUME_LEVEL8, 
    SPK_DIGITAL_VOLUME_LEVEL9,  
#endif    
    SPK_DIGITAL_VOLUME_LEVEL_NUM
};
typedef uint8_t spk_digital_volume_level_t;

enum
{
#if defined(CODEC_WM8960)
    MIC_AMP_GAIN_LEVEL0,    
    MIC_AMP_GAIN_LEVEL1,    
    MIC_AMP_GAIN_LEVEL2,    
    MIC_AMP_GAIN_LEVEL3,    
    MIC_AMP_GAIN_LEVEL4,    
    MIC_AMP_GAIN_LEVEL5,    
    MIC_AMP_GAIN_LEVEL6,    
    MIC_AMP_GAIN_LEVEL7,    
    MIC_AMP_GAIN_LEVEL8,    
    MIC_AMP_GAIN_LEVEL9,    
#endif
    MIC_AMP_GAIN_LEVEL_NUM 
};
typedef uint8_t mic_amp_gain_level_t;

enum
{
#if defined(CODEC_WM8960)
    MIC_DIGITAL_VOLUME_LEVEL0,    
    MIC_DIGITAL_VOLUME_LEVEL1,    
    MIC_DIGITAL_VOLUME_LEVEL2,    
    MIC_DIGITAL_VOLUME_LEVEL3,    
    MIC_DIGITAL_VOLUME_LEVEL4,    
    MIC_DIGITAL_VOLUME_LEVEL5,    
    MIC_DIGITAL_VOLUME_LEVEL6,    
    MIC_DIGITAL_VOLUME_LEVEL7,    
    MIC_DIGITAL_VOLUME_LEVEL8,    
    MIC_DIGITAL_VOLUME_LEVEL9,    
#endif  
    MIC_DIGITAL_VOLUME_LEVEL_NUM 
};
typedef uint8_t mic_digital_volume_level_t;

enum
{
    AUDIO_CODEC_SAMPLE_FREQ_8K, 
    AUDIO_CODEC_SAMPLE_FREQ_16K,    
    AUDIO_CODEC_SAMPLE_FREQ_32K,
    AUDIO_CODEC_SAMPLE_FREQ_44K,
    AUDIO_CODEC_SAMPLE_FREQ_48K,
    AUDIO_CODEC_SAMPLE_FREQ_MAX
};
typedef uint8_t sample_freq_t;

typedef struct
{
    uint8_t bitsPerSample;
    uint16_t sampleRate;
    uint8_t noOfChannels;
    uint8_t speaker_path;
    uint8_t mic_path;
    uint8_t interface_mode;     // interface master mode or not
    uint8_t interface_format;   // serial interface (pcm, i2s,...)
    uint8_t clk_mode;           // pll_mode or not
    uint8_t enable_mclk;

    uint8_t mic_amp_gain_level;
    uint8_t mic_digital_volume_level;

    uint8_t spk_amp_gain_level;
    uint8_t spk_digital_volume_level; 
} audioCodecCfg_t;

#ifdef CODEC_WM8960
    typedef uint16_t reg_val_t;
#else 
    typedef uint8_t reg_val_t;
#endif

#ifdef CODEC_WM8960
#define AudioCodec_read_reg(a)          WM8960_Read_Reg(a)                  
#define AudioCodec_write_reg(a, b)      WM8960_Write_Reg(a, b)  
#else
#define AudioCodec_read_reg(a)                          
#define AudioCodec_write_reg(a, b)      
#endif

void AudioCodec_init(void);
void AudioCodec_powerOn(uint8_t on);

/*
 *  codec open
 */
wm8960_result_t AudioCodec_open(void);
wm8960_result_t AudioCodec_reset(void);

#ifdef CODEC_WM8960
wm8960_result_t AudioCodec_config_new_wm8960(void);
#endif

/*
 * speaker volume up, down
 */
uint8_t AudioCodec_spkVol_up(void); 
uint8_t AudioCodec_spkVol_down(void); 
uint8_t AudioCodec_DAC_spkVol_up(void); 
uint8_t AudioCodec_DAC_spkVol_down(void); 

wm8960_result_t AudioCodec_spkAmpGainCtrl(uint8_t spkAmpGainLevel);
wm8960_result_t AudioCodec_spkDigitalVolCtrl(uint8_t volumeLevel);
wm8960_result_t AudioCodec_speakerMute(void);
wm8960_result_t AudioCodec_speakerUnmute(void);

/*
 * mic volume up, down
 */ 
uint8_t AudioCodec_micVol_up(void);
uint8_t AudioCodec_micVol_down(void);
uint8_t AudioCodec_ADC_micVol_up(void);
uint8_t AudioCodec_ADC_micVol_down(void);

wm8960_result_t AudioCodec_micAmpGainCtrl(uint8_t gain_level);
wm8960_result_t AudioCodec_micDigitalVolCtrl(uint8_t volumeLevel);
wm8960_result_t AudioCodec_micMute(void);
wm8960_result_t AudioCodec_micUnmute(void);

wm8960_result_t AudioCodec_setSampleFreq(sample_freq_t freq);


#if defined(CODEC_WM8960) && defined(ENABLE_WM8960_MMI_TEST)
extern void WM8960_register_check(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* AUDIOCODEC_H_ */
