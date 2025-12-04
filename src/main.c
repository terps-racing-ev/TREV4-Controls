#include "APDB.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

#include "utilities.h"
#include "can_manager.h"

#define CYCLE_TIME MsToUs(5ul)

/* Application Database,
 * needed for TTC-Downloader
 */
APDB appl_db =
          { 0                      /* ubyte4 versionAPDB        */
          , {0}                    /* BL_T_DATE flashDate       */
                                   /* BL_T_DATE buildDate                   */
          , { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
          , 0                      /* ubyte4 nodeType           */
          , 0                      /* ubyte4 startAddress       */
          , 0                      /* ubyte4 codeSize           */
          , 0                      /* ubyte4 legacyAppCRC       */
          , 0                      /* ubyte4 appCRC             */
          , 1                      /* ubyte1 nodeNr             */
          , 0                      /* ubyte4 CRCInit            */
          , 0                      /* ubyte4 flags              */
          , 0                      /* ubyte4 hook1              */
          , 0                      /* ubyte4 hook2              */
          , 0                      /* ubyte4 hook3              */
          , APPL_START             /* ubyte4 mainAddress        */
          , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
          , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
          , 0                      /* ubyte4 legacyHeaderCRC    */
          , 0                      /* ubyte4 version            */
          , BAUD_RATE              /* ubyte2 canBaudrate        */
          , 0                      /* ubyte1 canChannel         */
          , {0}                    /* ubyte1 reserved[8*4]      */
          , 0                      /* ubyte4 headerCRC          */
          };


void main (void)
{

    ubyte4 timestamp;

    /*******************************************/
    /*             INITIALIZATION              */
    /*******************************************/

    /* Initialize the IO driver (without safety functions) */
    IO_Driver_Init(NULL);

    /* NOTE: turns 5v sensor supply 1 on */
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    /* Set up FIFOs for all CAN messages */
    CAN_Manager_Init();

    

    /*******************************************/
    /*           MAIN CONTROL LOOP             */
    /*******************************************/    
    while (1)
    {
        IO_RTC_StartTime(&timestamp);
        IO_Driver_TaskBegin();

        /*******************************************/
        /*                 INPUTS                  */
        /*******************************************/
        //RTD_Update();
        //APPS_Update();
        //BSE_Update();
        CAN_Manager_ProcessRxMessages();  // Read CAN
        
        /*******************************************/
        /*                 LOGIC                   */
        /*******************************************/
        //StateMachine_Update();
        //TorqueController_Update();
        
        /*******************************************/
        /*                OUTPUTS                  */
        /*******************************************/
        //Outputs_Update();          // Control buzzer, lights, relay
        CAN_Manager_ProcessTxMessages();  // Send CAN        


        IO_Driver_TaskEnd();
        while (IO_RTC_GetTimeUS(timestamp) < CYCLE_TIME);
    }
}
