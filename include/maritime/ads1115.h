/*
 * ads1115.h
 * twitter: @ArduinoEasy
 *
 */

#ifndef ADS1115_H_
#define ADS1115_H_

#include <stdbool.h>
#include <stdint.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADS1115_ADDRESS (0x48) ///< 100 1000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
//#define ADS1115_CONVERSIONDELAY (1165) ///< Minimum Conversion Delay: usec ( Maximum Sample Rate: 860 SPS )
#define ADS1115_CONVERSIONDELAY (2) ///< Minimum Conversion Delay: msec ( Maximum Sample Rate: 860 SPS )
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK      (0x03)  ///< Point mask
#define ADS1115_REG_POINTER_CONVERT   (0x00)  ///< Conversion
#define ADS1115_REG_POINTER_CONFIG    (0x01)  ///< Configuration
#define ADS1115_REG_POINTER_LOWTHRESH (0x02)  ///< Low threshold
#define ADS1115_REG_POINTER_HITHRESH  (0x03)  ///< High threshold
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK    (0x8000) ///< OS Mask
#define ADS1115_REG_CONFIG_OS_SINGLE  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY    (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY (0x8000) ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK     (0x7000) ///< Mux Mask
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3

#define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3


// The ADC input range (or gain) can be changed via the following
// functions, but be careful never to exceed VDD +0.3V max, or to
// exceed the upper and lower limits if you adjust the input range!
// Setting these values incorrectly may destroy your ADC!
//                                                                              ADS1115
//                                                                              -------
// ADS1115_setGain(&adc1115, GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
// ADS1115_setGain(&adc1115, GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
// ADS1115_setGain(&adc1115, GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV  (default)
// ADS1115_setGain(&adc1115, GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
// ADS1115_setGain(&adc1115, GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
// ADS1115_setGain(&adc1115, GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

#define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00) ///< PGA Mask
#define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)   ///< Mode Mask
#define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000) ///< Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100) ///< Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_DR_MASK    (0x00E0)  ///< Data Rate Mask
#define ADS1115_REG_CONFIG_DR_8SPS    (0x0000)  ///< 8 samples per second
#define ADS1115_REG_CONFIG_DR_16SPS   (0x0020)  ///< 16 samples per second
#define ADS1115_REG_CONFIG_DR_32SPS   (0x0040)  ///< 32 samples per second
#define ADS1115_REG_CONFIG_DR_64SPS   (0x0060)  ///< 64 samples per second
#define ADS1115_REG_CONFIG_DR_128SPS  (0x0080)  ///< 128 samples per second (default)
#define ADS1115_REG_CONFIG_DR_250SPS  (0x00A0)  ///< 250 samples per second
#define ADS1115_REG_CONFIG_DR_475SPS  (0x00C0)  ///< 475 samples per second
#define ADS1115_REG_CONFIG_DR_860SPS  (0x00E0)  ///< 860 samples per second

#define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010) ///< CMode Mask
#define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008) ///< CPol Mask
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008) ///< ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000) ///< Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004) ///< Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK  (0x0003) ///< CQue Mask
#define ADS1115_REG_CONFIG_CQUE_1CONV (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/** Gain settings */
typedef enum {
  GAIN_TWOTHIRDS = ADS1115_REG_CONFIG_PGA_6_144V,
  GAIN_ONE       = ADS1115_REG_CONFIG_PGA_4_096V,
  GAIN_TWO       = ADS1115_REG_CONFIG_PGA_2_048V,
  GAIN_FOUR      = ADS1115_REG_CONFIG_PGA_1_024V,
  GAIN_EIGHT     = ADS1115_REG_CONFIG_PGA_0_512V,
  GAIN_SIXTEEN   = ADS1115_REG_CONFIG_PGA_0_256V
} adsGain_t;

/** Sampling settings */
typedef enum {
  SPS_8   = ADS1115_REG_CONFIG_DR_8SPS,
  SPS_16  = ADS1115_REG_CONFIG_DR_16SPS,
  SPS_32  = ADS1115_REG_CONFIG_DR_32SPS,
  SPS_64  = ADS1115_REG_CONFIG_DR_64SPS,
  SPS_128 = ADS1115_REG_CONFIG_DR_128SPS,
  SPS_250 = ADS1115_REG_CONFIG_DR_250SPS,
  SPS_475 = ADS1115_REG_CONFIG_DR_475SPS,
  SPS_860 = ADS1115_REG_CONFIG_DR_860SPS
} adsSPS_t;

/** Comparator Mode */
typedef enum {
  CMODE_TRAD  = ADS1115_REG_CONFIG_CMODE_TRAD,
  CMODE_WINDOW  = ADS1115_REG_CONFIG_CMODE_WINDOW
} adsCMODE_t;

/** Conversion Mode */
typedef enum {
  SINGLE_CONV = ADS1115_REG_CONFIG_MODE_SINGLE,
  CONT_CONV   = ADS1115_REG_CONFIG_MODE_CONTIN
} adsCONV_t;

/** ADC channels */
typedef enum {
  CH_0  = ADS1115_REG_CONFIG_MUX_SINGLE_0,
  CH_1  = ADS1115_REG_CONFIG_MUX_SINGLE_1,
  CH_2  = ADS1115_REG_CONFIG_MUX_SINGLE_2,
  CH_3  = ADS1115_REG_CONFIG_MUX_SINGLE_3,

  DIFF_0_1 = ADS1115_REG_CONFIG_MUX_DIFF_0_1,
  DIFF_0_3 = ADS1115_REG_CONFIG_MUX_DIFF_0_3,
  DIFF_1_3 = ADS1115_REG_CONFIG_MUX_DIFF_1_3,
  DIFF_2_3 = ADS1115_REG_CONFIG_MUX_DIFF_2_3
} adc_Ch_t;

/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit ADS1115 ADC breakout.
*/
/**************************************************************************/

/*  Structure to store address and settings of ADS1115 16-bit ADC IC  */
typedef struct
{
  // Instance-specific properties
  uint16_t    config;             //< ADC config
  uint16_t    m_samplingRate;     //< sampling rate
  uint16_t    m_mode;             //< ADC mode
  uint32_t    m_conversionDelay;  //< conversion deay
  adsGain_t   m_gain;             //< ADC gain
  uint16_t    m_compMode;         //< Comparator Mode
  float       m_lsbMultiplier;    //< LSB multiplier
  uint16_t    Hi_thresh;          //< High Threshold value
  uint16_t    Lo_thresh;          //< Low Threshold value
}ADS1115;


void ADS1115_reset(ADS1115 *ads1115_module);

void ADS1115_init(ADS1115 *ads1115_module);

float ADS1115_readADC(ADS1115 *ads1115_module, adc_Ch_t channel);
int16_t ADS1115_readADC_raw(ADS1115 *ads1115_module, adc_Ch_t channel);

void ADS1115_startComparator_SingleEnded(ADS1115 *ads1115_module, uint8_t channel, int16_t threshold);

float ADS1115_getLastConversionResults(ADS1115 *ads1115_module);

void ADS1115_setGain(ADS1115 *ads1115_module, adsGain_t gain);
adsGain_t ADS1115_getGain(ADS1115 *ads1115_module);

void ADS1115_setSPS(ADS1115 *ads1115_module, adsSPS_t sps);
adsSPS_t ADS1115_getSPS(ADS1115 *ads1115_module);

void ADS1115_setCONV(ADS1115 *ads1115_module, adsCONV_t sps);
adsCONV_t ADS1115_getCONV(ADS1115 *ads1115_module);

#endif /* ADS1115_H_ */