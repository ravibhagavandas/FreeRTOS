/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#include "FreeRTOS.h"
#include "task.h"
#include "delay-board.h"

void DelayMsMcu( uint32_t ms )
{
    vTaskDelay( pdMS_TO_TICKS(ms) );
   //HAL_Delay( ms );
}
