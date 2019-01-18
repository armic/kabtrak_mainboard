/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include "system.h"
#include "usb_device.h"
#include "Board_LEDs.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Data Types
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Macros or Functions
// *****************************************************************************
// *****************************************************************************

void APP_LEDUpdateUSBStatus(void)
{
    static uint16_t ledCount = 0;
    static uint8_t  GlowCount = 0;

    if(USBIsDeviceSuspended() == true)
    {
        Set_LED_Off(Green_LED);
        return;
    }

    switch(USBGetDeviceState())
    {         
        case CONFIGURED_STATE:
            
            /* We are configured.  Slow brightening and darkening.. */
            if (GlowCount > 81)
            {
                GlowCount = 0;
            }
            
            if (ledCount > 20)
            {
                ledCount = 0;
                GlowCount++;
            }
            
            if (GlowCount <= 40)
            {
                if (ledCount < (GlowCount/2))
                {
                    Set_LED_On(Green_LED);
                }
                else {
                    Set_LED_Off(Green_LED);
                }
                
            }
            else {
                if (ledCount <= (GlowCount-41)/2)
                {
                    Set_LED_Off(Green_LED);
                }
                else {
                    Set_LED_On(Green_LED);
                    
                }
            }
            
            
            break;

        default:
            /* We aren't configured yet, but we aren't suspended so let's blink with
             * a slow pulse. On for 50ms, then off for 950ms, then reset/repeat. */
            if(ledCount == 1)
            {
                Set_LED_On(Green_LED);
            }
            else if(ledCount == 50)
            {
                Set_LED_Off(Green_LED);
            }
            else if(ledCount > 950)
            {
                ledCount = 0;
            }
            break;
    }

    /* Increment the millisecond counter. */
    ledCount++;
}

/*******************************************************************************
 End of File
*/
