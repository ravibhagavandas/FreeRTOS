#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "gpio.h"
#include "timer.h"
#include "LoRaMac.h"

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_ADR_ON                              1
#define LORAWAN_APP_DATA_MAX_SIZE                   242
#define LORAWAN_APP_PORT                            2
#define LORAWAN_PUBLIC_NETWORK                      1

#define MAX_JOIN_ATTEMPTS          6

#define APP_SEND_CONFIRMED_MESSAGES                 ( 1 )

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

#define DEV_EUI  { 0x32, 0x38, 0x33, 0x35, 0x60, 0x38, 0x71, 0x01 };
#define JOIN_EUI { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0xD1, 0xD4 };
#define APP_NWK_KEY { 0xF5, 0x42, 0x96, 0x98, 0x8B, 0xC2, 0x23, 0x86, 0x56, 0x24, 0x1D, 0x73, 0x0A, 0xFA, 0x95, 0x0B };

const char* MacStatusStrings[] =
{
    "OK",                            // LORAMAC_STATUS_OK
    "Busy",                          // LORAMAC_STATUS_BUSY
    "Service unknown",               // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",             // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",             // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",              // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid", // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",             // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                  // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",          // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",              // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",         // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",              // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",         // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",     // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",    // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",         // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                  // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",            // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",             // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                  // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",           // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Multicast group undefined",     // LORAMAC_STATUS_MC_GROUP_UNDEFINED
    "Unknown error",                 // LORAMAC_STATUS_ERROR
};

const char* EventInfoStatusStrings[] =
{ 
    "OK",                            // LORAMAC_EVENT_INFO_STATUS_OK
    "Error",                         // LORAMAC_EVENT_INFO_STATUS_ERROR
    "Tx timeout",                    // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT
    "Rx 1 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
    "Rx 2 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
    "Rx1 error",                     // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
    "Rx2 error",                     // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
    "Join failed",                   // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL
    "Downlink repeated",             // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED
    "Tx DR payload size error",      // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR
    "Downlink too many frames loss", // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS
    "Address fail",                  // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL
    "MIC fail",                      // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL
    "Multicast fail",                // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL
    "Beacon locked",                 // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED
    "Beacon lost",                   // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST
    "Beacon not found"               // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND
};
 
 typedef enum
 {
	 LORAWAN_JOIN_MODE_OTAA = 0,
	 LORAWAN_JOIN_MODE_ABP
 } LoraWanJoinMode_t;


/*!
 * Device states
 */
typedef enum eDeviceState
{
    DEVICE_STATE_RESTORE,
    DEVICE_STATE_START,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState_t;

DeviceState_t DeviceState;
LoRaMacPrimitives_t LoRaMacPrimitives;
LoRaMacCallback_t LoRaMacCallbacks;

static uint8_t appDataBuffer[LORAWAN_APP_DATA_MAX_SIZE];
TimerEvent_t Led1_Timer;  //Lora join indicator light
TimerEvent_t Led2_Timer;  //LoRa send out indicator light
static TimerEvent_t packetTimer;

void OnLed2_TimerEvent(void *context)
{  
    TimerStop(&Led2_Timer);
    GpioWrite(&Led2, 1);
}

void OnLed1_TimerEvent(void * context)
{
    TimerStop(&Led1_Timer);
    GpioWrite(&Led1, 1);
}



void ledBlink(void)
{
    
    DelayMs(200);
    
    GpioWrite(&Led1,0);
    GpioWrite(&Led2,0);
    DelayMs(200);
    
    GpioWrite(&Led1,1);
    GpioWrite(&Led2,1);
    
    DelayMs(200);
}

void peripheralInit( void )
{
    //TODO: Enable GPS later
    //GpsInit( );
     //LIS3DH_Init( );
     //BME680_Init( );
}

static LoRaMacStatus_t joinNetwork( void )
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;
    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

    // Starts the join procedure
    status = LoRaMacMlmeRequest( &mlmeReq );

    if( status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED )
    {
        e_printf( "Next Tx in  : ~%lu second(s)\n", ( mlmeReq.ReqReturn.DutyCycleWaitTime / 1000 ) );
    }
    
    return status;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnPacketTimerEvent( void* context )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &packetTimer );

    mibReq.Type = MIB_NETWORK_ACTIVATION;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
        {
            // Network not joined yet. Try to join again.
            DeviceState = DEVICE_STATE_JOIN;
        }
        else
        {
            DeviceState = DEVICE_STATE_SEND;
        }
    }
}

static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    e_printf( "\n MCPS CONFIRM status: %s\n", EventInfoStatusStrings[mcpsConfirm->Status] );
    if( mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        DeviceState = DEVICE_STATE_SLEEP;
    }
    else
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                e_printf("Unconfirmed message sent out.\n");
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                if( mcpsConfirm->AckReceived == true )
                {
                    e_printf("ACK received for confirmed message.\n");
                    DeviceState = DEVICE_STATE_CYCLE;
                }
                else
                {
                    e_printf("NACK received for confirmed message.\n");
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        //GpioWrite( &Led1, 1 );
        //TimerStart( &Led1Timer );
    }
}

/*!
 * Prints the provided buffer in HEX
 * 
 * \param buffer Buffer to be printed
 * \param size   Buffer size to be printed
 */
static void PrintHexBuffer( uint8_t *buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            e_printf( "\n" );
            newline = 0;
        }

        e_printf( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    e_printf( "\n" );
}
 

static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    e_printf( "\nMCPS INDICATION status: %s\n", EventInfoStatusStrings[mcpsIndication->Status] );
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnPacketTimerEvent( NULL );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if( mcpsIndication->RxData == true )
    {
       e_printf( "Downlink data received: port: %d, slot: %d, data_rate:%d, rssi: %d, snr:%d data:\n",
       mcpsIndication->Port,
       mcpsIndication->RxSlot,
       mcpsIndication->RxDatarate,
       mcpsIndication->Rssi,
       mcpsIndication->Snr );
       PrintHexBuffer( mcpsIndication->Buffer, mcpsIndication->BufferSize );
    }
}


static void MlmeIndication ( MlmeIndication_t* MlmeIndication )
{
   // Implementation for MLME indication
}

 
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    LoRaMacEventInfoStatus_t status = mlmeConfirm->Status;
    MibRequestConfirm_t mibGet;
    static int nJoinAttempts = MAX_JOIN_ATTEMPTS;


	e_printf("MLME CONFIRM  status: %s\n", EventInfoStatusStrings[status]);

	switch( mlmeConfirm->MlmeRequest )
	{
		case MLME_JOIN:
        {

            if( status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                e_printf("OTAA JOIN Successful\n\n");
                
                mibGet.Type = MIB_DEV_ADDR;
                LoRaMacMibGetRequestConfirm( &mibGet );
                e_printf( "DevAddr : %08lX\n", mibGet.Param.DevAddr );

                mibGet.Type = MIB_CHANNELS_DATARATE;
                LoRaMacMibGetRequestConfirm( &mibGet );
                e_printf( "DATA RATE   : DR_%d\n", mibGet.Param.ChannelsDatarate );

                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                if( nJoinAttempts > 0 )
                {
                    e_printf("Join Failed, max join attempts left: %d\n", nJoinAttempts );
                    nJoinAttempts--;
                    DeviceState = DEVICE_STATE_CYCLE;
                }
                else
                {
                    e_printf( "Join Failed, no attempts left..\n" );
                    DeviceState = DEVICE_STATE_SLEEP;
                }
            }
            break;
        }

		default:
            break;
	}
}

static void OnMacProcessNotify( void )
{
    //IsMacProcessPending = 1;
}
uint8_t  getBatteryLevel( void )
{
	return 0;
}

static LoRaMacStatus_t configure( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;
    uint8_t devEUI[8] = DEV_EUI;
    uint8_t joinEUI[8] = JOIN_EUI;
    uint8_t appKey[16] = APP_NWK_KEY;

    mibReq.Type = MIB_DEV_EUI;
    mibReq.Param.DevEui = devEUI;
    status = LoRaMacMibSetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        mibReq.Type = MIB_JOIN_EUI;
        mibReq.Param.JoinEui = joinEUI;
        status = LoRaMacMibSetRequestConfirm( &mibReq );

    }

    if( status == LORAMAC_STATUS_OK )
    {
        mibReq.Type = MIB_APP_KEY;
        mibReq.Param.AppKey = appKey;
        status = LoRaMacMibSetRequestConfirm( &mibReq );
    }

    if( status == LORAMAC_STATUS_OK )
    {
        mibReq.Type = MIB_NWK_KEY;
        mibReq.Param.AppKey = appKey;
        status = LoRaMacMibSetRequestConfirm( &mibReq );
    }
    
    if( status == LORAMAC_STATUS_OK )
    {            
        mibReq.Type = MIB_PUBLIC_NETWORK;
        mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
        LoRaMacMibSetRequestConfirm( &mibReq );
    }

    if( status == LORAMAC_STATUS_OK )
    {
        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
        status = LoRaMacMibSetRequestConfirm( &mibReq );
    }


#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
    if( status == LORAMAC_STATUS_OK )
    {
        status = LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
    }
#endif

    if( status == LORAMAC_STATUS_OK )
    {
        mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
        mibReq.Param.SystemMaxRxError = 20;
        status = LoRaMacMibSetRequestConfirm( &mibReq );
    }

    return status;
}


/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [1: frame could be send, 0: error]
 */
static LoRaMacStatus_t sendFrame( bool confirmed )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    uint32_t appDataSize;
    LoRaMacStatus_t status;

    appDataBuffer[0] = 0xFF;
    appDataSize = 1;

    if( LoRaMacQueryTxPossible( appDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        e_printf("TX not possible for data size.\n");
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( confirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = LORAWAN_APP_PORT;
            mcpsReq.Req.Unconfirmed.fBuffer = appDataBuffer;
            mcpsReq.Req.Unconfirmed.fBufferSize = appDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = LORAWAN_APP_PORT;
            mcpsReq.Req.Confirmed.fBuffer = appDataBuffer;
            mcpsReq.Req.Confirmed.fBufferSize = appDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    status = LoRaMacMcpsRequest( &mcpsReq );
    
    e_printf( "MCPS-Request status : %s\n", MacStatusStrings[status] );

    if( status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED )
    {
        e_printf( "TX: Duty cycle limit reached.\n");
    }

    return status;
}

extern void GPIOIRQ_Enable(void);

static void ledBlinkLoopTask(void * params)
{
    uart_config_t uart_config = DEFAULT_VALUE; 

    UartMcuInit(&Uart1, 1, UART_TX, UART_RX);
    UartMcuConfig(&Uart1, RX_TX, uart_config.baudrate, 
                                      uart_config.wordLength,
                                      uart_config.stopBits,
                                      uart_config.parity,
                                      uart_config.flowCtrl);
    
  while(1)
  {
    
    GpioWrite(&Led1,1);
    GpioWrite(&Led2,1);
    DelayMs(1000);
    
    e_printf("HelloWorld!\r\n");
    
    GpioWrite(&Led1,0);
    GpioWrite(&Led2,0);
        
    DelayMs(1000);
  }
  
  vTaskDelete(NULL);
}
    

static void prvLorawanClassATask( void *params )
{
    LoRaMacStatus_t status;
    uint32_t dutyCycleTime = 0;
    MibRequestConfirm_t mibReq;
    uart_config_t uart_config = DEFAULT_VALUE; 
    
    ledBlink();
    peripheralInit();
    
    DelayMs(1000);

    UartMcuInit(&Uart1, 1, UART_TX, UART_RX);
    UartMcuConfig(&Uart1, RX_TX, uart_config.baudrate, 
                                      uart_config.wordLength,
                                      uart_config.stopBits,
                                      uart_config.parity,
                                      uart_config.flowCtrl);
    
    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
    LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
    LoRaMacCallbacks.GetBatteryLevel = getBatteryLevel;
    
    status = LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915);
    if( status == LORAMAC_STATUS_OK )
    {
      e_printf("Lora MAC initialization successfull.\r\n");
    }
    else
    {
        e_printf("Lora MAC initialization failed, %d.\r\n", status);
        for(;;);
    }

    GPIOIRQ_Enable();

    DeviceState = DEVICE_STATE_RESTORE;

    e_printf( "###### ===== Class A demo application v1.0.0 ==== ######\n\n" );
    
    while( 1 )
    {
       // Process Radio IRQ.
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }

        //Process Lora mac events.
        LoRaMacProcess( );

        switch( DeviceState )
        {
        case DEVICE_STATE_RESTORE:
            status = configure();
            if( status == LORAMAC_STATUS_OK )
	        {
		        e_printf("Lora MAC configuration successfull \r\n");
                DeviceState = DEVICE_STATE_START;
	        }
            else
            {
                e_printf("LoraMAC config failed. going to sleep");
                DeviceState = DEVICE_STATE_SLEEP;
            }
            break;
        case DEVICE_STATE_START:
            TimerInit( &packetTimer, OnPacketTimerEvent );
            status =  LoRaMacStart( );
            if( status == LORAMAC_STATUS_OK )
	        {
                e_printf("Lora MAC start successfull \r\n");

                mibReq.Type = MIB_NETWORK_ACTIVATION;
                status = LoRaMacMibGetRequestConfirm( &mibReq );
                if( status == LORAMAC_STATUS_OK )
                {
                    if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
                    {
                        e_printf("Initiating join... \r\n");
                        DeviceState = DEVICE_STATE_JOIN;
                    }
                    else
                    {
                        e_printf("Already joined.Sending data \r\n");

                        DeviceState = DEVICE_STATE_SEND;
                    }
                }
                else
                {
                    e_printf("Error while fetching activation status. %d", status );
                    DeviceState = DEVICE_STATE_SLEEP;
                } 
            }
            else
            {
                e_printf("LoraMAC start failed. going to sleep");
                DeviceState = DEVICE_STATE_SLEEP;
            }
            
            break;
        case DEVICE_STATE_JOIN:
            status = joinNetwork();
            if( status == LORAMAC_STATUS_OK )
            {
                e_printf("OTA Joining... \n");
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                if( status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED )
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            
            break;
        case DEVICE_STATE_SEND:
            status = sendFrame(APP_SEND_CONFIRMED_MESSAGES);
            if( status == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                if( status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED )
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            
            break;
        case DEVICE_STATE_CYCLE:
            // Schedule next packet transmission
            DeviceState = DEVICE_STATE_SLEEP;
            dutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
            // Schedule next packet transmission
            TimerSetValue( &packetTimer, dutyCycleTime );
            TimerStart( &packetTimer );
            break;
		default:
            break;
        }
    }
    
    vTaskDelete(NULL);
}

/*******************************************************************************************
 * the app_main function
 * *****************************************************************************************/ 
void main(void)
{
  
    BoardInitMcu();
	
    //xTaskCreate( ledBlinkLoopTask, "LoRa", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL ); 
    xTaskCreate( prvLorawanClassATask, "LoRa", configMINIMAL_STACK_SIZE * 40 , NULL, tskIDLE_PRIORITY + 1, NULL );    
    
    /* Start the scheduler. */
    vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );

	
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

	/* Parameters are not used. */
	( void ) ulLine;
	( void ) pcFileName;

	taskENTER_CRITICAL();
	{
		while( ulSetToNonZeroInDebuggerToContinue == 0 )
		{
			/* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
			non zero value to step out of this function to the point that raised
			this assert(). */
			__asm volatile( "NOP" );
			__asm volatile( "NOP" );
		}
	}
	taskEXIT_CRITICAL();
}

