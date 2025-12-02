/**************************************************************************
 *  XC-2000 IO-LIB
 *
 *  Template
 **************************************************************************
 *
 *  Quickstart Template for HY-TTC60
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "IO_CAN.h"
#include "APDB.h"
#include "math.h"

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

    /*******************************************/
    /*             INITIALIZATION              */
    /*******************************************/

    /* Initialize the IO driver (without safety functions) */
    IO_Driver_Init(NULL);

    /*  - Use the IO driver functions to initialize */
    /*    all required IOs and interfaces           */

    /* NOTE: turns 5v sensor supply 1 on */
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/

    /* main loop, executed periodically with a
     * defined cycle time (here: CYCLE_TIME_MS ms)
     */
    
    ubyte4 timestamp;
    while (1)
    {
        /* get a timestamp to implement the cycle time */
        IO_RTC_StartTime(&timestamp);
        /* Task begin function for IO Driver
         * This function needs to be called at
         * the beginning of every SW cycle
         */
        IO_Driver_TaskBegin();

        // LOOP CODE

        /* Task end function for IO Driver
         * This function needs to be called at
         * the end of every SW cycle
         */
        IO_Driver_TaskEnd();

        while (IO_RTC_GetTimeUS(timestamp) < CYCLE_TIME);
    }
}
