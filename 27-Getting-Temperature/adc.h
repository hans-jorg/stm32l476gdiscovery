#ifndef ADC_H
#define ADC_H

#define ADC1_VBAT_CHANNEL    18
#define ADC3_VBAT_CHANNEL    18
#define ADC1_TEMP_CHANNEL    17
#define ADC2_TEMP_CHANNEL    17
#define ADC1_VREF_CHANNEL    0
#define ADC2_DAC1_CHANNEL    17
#define ADC3_DAC1_CHANNEL    14
#define ADC2_DAC2_CHANNEL    18
#define ADC3_DAC2_CHANNEL    15


int     ADC_Init(ADC_TypeDef *adc);
int     ADC_ConfigureChannel(ADC_TypeDef *adc, uint32_t ch, uint32_t config);
int32_t ADC_Read(ADC_TypeDef *adc, uint32_t ch);
int     ADC_ReadMultiple(ADC_TypeDef *adc, uint32_t *ch, uint32_t *val, int n);
int32_t ADC_ReadTemperature(void);
int32_t ADC_ReadVBat(void);
int32_t ADC_ReadVREF(void);


// Config values in ADC_ConfigChannel
// Mode: bit 3:0
#define ADC_MODE_SAMPLING_2_5   0
#define ADC_MODE_SAMPLING_6_5   1
#define ADC_MODE_SAMPLING_12_5  2
#define ADC_MODE_SAMPLING_25_5  3
#define ADC_MODE_SAMPLING_47_5  4
#define ADC_MODE_SAMPLING_92_5  5
#define ADC_MODE_SAMPLING_247_5 6
#define ADC_MODE_SAMPLING_640_5 7
// Differential: bit 8
#define ADC_MODE_DIFFERENTIAL   256
#define ADC_MODE_SINGLE_ENDED   0


#endif
