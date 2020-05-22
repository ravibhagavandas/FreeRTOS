/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board I2C driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "i2c-board.h"

/*!
 *  The value of the maximal timeout for I2C waiting loops
 */
#define TIMEOUT_MAX                                 0x8000

static I2cAddrSize I2cInternalAddrSize = I2C_ADDR_SIZE_8;

static I2C_HandleTypeDef I2CHandle = { 0 };

void I2cMcuInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
    __HAL_RCC_I2C1_CLK_DISABLE( );
    __HAL_RCC_I2C1_CLK_ENABLE( );
    __HAL_RCC_I2C1_FORCE_RESET( );
    __HAL_RCC_I2C1_RELEASE_RESET( );

    obj->I2cId = i2cId;

    I2CHandle.Instance  = ( I2C_TypeDef * )I2C1_BASE;

    GpioInit( &obj->Scl, scl, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, GPIO_AF4_I2C1 );
    GpioInit( &obj->Sda, sda, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, GPIO_AF4_I2C1 );
}

void I2cMcuFormat( I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency )
{
    __HAL_RCC_I2C1_CLK_ENABLE( );
    I2CHandle.Init.ClockSpeed = I2cFrequency;

    if( dutyCycle == I2C_DUTY_CYCLE_2 )
    {
        I2CHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    }
    else
    {
        I2CHandle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    }

    I2CHandle.Init.OwnAddress1 = 0;
    I2CHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2CHandle.Init.OwnAddress2 = 0;
    I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2CHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_I2C_Init( &I2CHandle );
}

void I2cMcuDeInit( I2c_t *obj )
{

    HAL_I2C_DeInit( &I2CHandle );


    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE( );

    GpioInit( &obj->Scl, obj->Scl.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Sda, obj->Sda.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
    I2cInternalAddrSize = addrSize;
}

uint8_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Write( &I2CHandle, deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? SUCCESS : FAIL;
    return status;
}

uint8_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Read( &I2CHandle, deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? SUCCESS : FAIL;
    return status;
}

uint8_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr )
{
    uint8_t status = FAIL;
    status = ( HAL_I2C_IsDeviceReady( &I2CHandle, deviceAddr, 300, 4096 ) == HAL_OK ) ? SUCCESS : FAIL;;
    return status;
}


void I2cMcuResetBus( I2c_t *obj )
{
    I2cInit( obj, obj->I2cId, I2C_SCL, I2C_SDA );
}