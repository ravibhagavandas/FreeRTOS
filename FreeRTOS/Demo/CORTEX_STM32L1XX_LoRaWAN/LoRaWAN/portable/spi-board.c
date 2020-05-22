/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "spi-board.h"
#include "stm32l1xx_hal_spi.h"
#include "stm32l1xx_hal_gpio_ex.h"

/*!
 * \brief  Find First Set
 *         This function identifies the least significant index or position of the
 *         bits set to one in the word
 *
 * \param [in]  value  Value to find least significant index
 * \retval bitIndex    Index of least significat bit at one
 */
__STATIC_INLINE uint8_t __ffs( uint32_t value )
{
    return( uint32_t )( 32 - __CLZ( value & ( -value ) ) );
}


static SPI_HandleTypeDef SpiHandle[2];


void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    
    obj->SpiId = spiId;

    if( obj->SpiId == SPI_1 )
    {
        __HAL_RCC_SPI1_FORCE_RESET( );
        __HAL_RCC_SPI1_RELEASE_RESET( );

        __HAL_RCC_SPI1_CLK_ENABLE( );

        SpiHandle[spiId].Instance = ( SPI_TypeDef* )SPI1_BASE;
        

        GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
        GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
        GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
            if( nss != NC )
        {
            GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF5_SPI1 );
        }
        
    }
    else
    {
        __HAL_RCC_SPI2_FORCE_RESET( );
        __HAL_RCC_SPI2_RELEASE_RESET( );
        __HAL_RCC_SPI2_CLK_ENABLE( );

        SpiHandle[spiId].Instance = ( SPI_TypeDef* )SPI2_BASE;

        GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
        GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
        GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
        if( nss != NC )
        {
            GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF5_SPI2 );
        }
    }

    if( nss == NC )
    {
        SpiHandle[spiId].Init.NSS = SPI_NSS_SOFT;
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 0 );
    }
    else
    {
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 1 );
    }
    SpiFrequency( obj, 1000000 );

    HAL_SPI_Init( &SpiHandle[spiId] );
}

void SpiDeInit( Spi_t *obj )
{
    HAL_SPI_DeInit( &SpiHandle[obj->SpiId] );

    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    SpiHandle[obj->SpiId].Init.Direction = SPI_DIRECTION_2LINES;
    if( bits == SPI_DATASIZE_8BIT )
    {
        SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_8BIT;
    }
    else
    {
        SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_16BIT;
    }
    SpiHandle[obj->SpiId].Init.CLKPolarity = cpol;
    SpiHandle[obj->SpiId].Init.CLKPhase = cpha;
    SpiHandle[obj->SpiId].Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle[obj->SpiId].Init.TIMode = SPI_TIMODE_DISABLE;
    SpiHandle[obj->SpiId].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle[obj->SpiId].Init.CRCPolynomial = 7;

    if( slave == 0 )
    {
        SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_MASTER;
    }
    else
    {
        SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_SLAVE;
    }
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    uint32_t divisor;

    divisor = SystemCoreClock / hz;

    // Find the nearest power-of-2
    divisor = divisor > 0 ? divisor-1 : 0;
    divisor |= divisor >> 1;
    divisor |= divisor >> 2;
    divisor |= divisor >> 4;
    divisor |= divisor >> 8;
    divisor |= divisor >> 16;
    divisor++;

    divisor = __ffs( divisor ) - 1;

    divisor = ( divisor > 0x07 ) ? 0x07 : divisor;

    SpiHandle[obj->SpiId].Init.BaudRatePrescaler = divisor << 3;
}

FlagStatus SpiGetFlag( Spi_t *obj, uint16_t flag )
{
    FlagStatus bitstatus = RESET;

    // Check the status of the specified SPI flag
    if( ( SpiHandle[obj->SpiId].Instance->SR & flag ) != ( uint16_t )RESET )
    {
        // SPI_I2S_FLAG is set
        bitstatus = SET;
    }
    else
    {
        // SPI_I2S_FLAG is reset
        bitstatus = RESET;
    }
    // Return the SPI_I2S_FLAG status
    return  bitstatus;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    uint8_t rxData = 0;

    if( ( obj == NULL ) || ( SpiHandle[obj->SpiId].Instance ) == NULL )
    {
        assert_param( FAIL );
    }
    __HAL_SPI_ENABLE( &SpiHandle[obj->SpiId] );

    while( SpiGetFlag( obj, SPI_FLAG_TXE ) == RESET );
    SpiHandle[obj->SpiId].Instance->DR = ( uint16_t ) ( outData & 0xFF );

    while( SpiGetFlag( obj, SPI_FLAG_RXNE ) == RESET );
    rxData = ( uint16_t ) SpiHandle[obj->SpiId].Instance->DR;

    return( rxData );
}

