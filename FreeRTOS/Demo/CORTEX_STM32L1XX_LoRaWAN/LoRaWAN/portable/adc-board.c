/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Board ADC driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Andreas Pella (IMST GmbH), Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "adc-board.h"

/*!
 * Calibration Data Bytes base address for medium density devices
 */
#define FACTORY_TSCALIB_BASE                        ( ( uint32_t )0x1FF80078 )
#define PDDADC_AVG_SLOPE                            1610 // 1.61 * 1000
#define PDDADC_OVERSAMPLE_FACTOR                    0x04

ADC_HandleTypeDef AdcHandle;

void AdcMcuInit( Adc_t *obj, PinNames adcInput )
{
     AdcHandle.Instance = ( ADC_TypeDef* )ADC1_BASE;

    GpioInit( &obj->AdcInput, adcInput, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void AdcMcuConfig( void )
{
    //TODO: Check and enable configuration for the board.
#if 0
    // Configure ADC
    AdcHandle.Init.OversamplingMode      = DISABLE;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    AdcHandle.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerFrequencyMode = DISABLE; // To be enabled only if ADC clock < 2.8 MHz
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    HAL_ADC_Init( &AdcHandle );

    // Calibration
    HAL_ADCEx_Calibration_Start( &AdcHandle, ADC_SINGLE_ENDED );

#endif

}


uint16_t AdcMcuRead( Adc_t *obj, uint8_t channel )
{
        ADC_ChannelConfTypeDef adcConf;
        uint16_t adcData = 0;

            /* Enable HSI */
        __HAL_RCC_HSI_ENABLE();

        /* Wait till HSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
        {
        }

        __HAL_RCC_ADC1_CLK_ENABLE( );

        adcConf.Channel = channel;
        adcConf.Rank = ADC_REGULAR_RANK_1;
        adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;

        HAL_ADC_ConfigChannel( &AdcHandle, &adcConf);

        /* Enable ADC1 */
        __HAL_ADC_ENABLE( &AdcHandle) ;

        /* Start ADC1 Software Conversion */
        HAL_ADC_Start( &AdcHandle);

        HAL_ADC_PollForConversion( &AdcHandle, HAL_MAX_DELAY );

        adcData = HAL_ADC_GetValue ( &AdcHandle);

        __HAL_ADC_DISABLE( &AdcHandle) ;

        if( ( adcConf.Channel == ADC_CHANNEL_TEMPSENSOR ) || ( adcConf.Channel == ADC_CHANNEL_VREFINT ) )
        {
                HAL_ADC_DeInit( &AdcHandle );
        }
        __HAL_RCC_ADC1_CLK_DISABLE( );

        /* Disable HSI */
        __HAL_RCC_HSI_DISABLE();

        return adcData;
}
