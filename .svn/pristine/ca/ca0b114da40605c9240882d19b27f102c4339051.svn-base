/*******************************************************************************
  
  
  
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system.h"

#include "usb.h"
#include "usb_device_hid.h"

#include <stdint.h>
#include <stdbool.h>        /* For true/false definition */


#include "Timer.h"
#include "Cabinet_Interface.h"
#include "Draw_Interface.h"
#include "Board_LEDs.h"
#include "USB_Interface.h"
#include "Debug_UART.h"


// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
const uint8_t FirmwareVersionNumber[2] = { 0x04/*Major*/, 0x00/* Minor */ }; 

// *****************************************************************************
// Section: Function definition
// *****************************************************************************



/******************************************************************************
 * Function:        int main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets CPU clock and initializes i2c.
 *****************************************************************************/
int main ( void )
{
    uint16_t LoopCounter;
    uint16_t Ms_Timer_Check;
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);
    
    Set_7Segment_Char('0');
    
    Init_Timer1();          // opens with set config bytes
    
    Init_Debug_UART();
    Debug_Print("Starting\n\r");
    
    USBDeviceInit();
    USBDeviceAttach();
    
    
    Cabinet_Interface_Init();
    ClrWdt();
    
    
    Draw_Interface_Init();
    ClrWdt();
    
   
    
    Ms_Timer_Check = Get_Timer_Count();
    LoopCounter = 0;
    Set_LED_Off(DotPoint_LED);
            

    while( 1 )
    {
        if (Ms_Timer_Check != Get_Timer_Count())
        {
            Ms_Timer_Check = Get_Timer_Count();
            
            Cabinet_Interface_Ms_Poll();
            if (++LoopCounter > 100)
            {
                LoopCounter = 0;
                
                /* This section defines the tasks which must be run exactly every 100ms for timing purposes.*/
                ClrWdt(); /* Kick the watchdog */
                Kick_Ext_WatchDog();

                Step_7Segment_Loop2();
                Toggle_LED(DotPoint_LED);
                Cabinet_Interface_100ms_Poll();
                Draw_Interface_100ms_Poll();
            }
        }
        
        //Application specific tasks
        USB_Interface_Fast_Poll();
        
        Draw_Interface_Fast_Poll();
    }
}





/*******************************************************************************
 End of File
*/
