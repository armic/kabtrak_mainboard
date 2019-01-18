/** INCLUDES *******************************************************/
#include "USB_Interface.h"
#include "usb.h"
#include "usb_device_hid.h"

#include <string.h>

#include "system.h"
#include "Board_LEDs.h"

#include "Timer.h"
#include "Cabinet_Interface.h"
#include "Draw_Interface.h"


/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata HID_CUSTOM_OUT_DATA_BUFFER = HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS
        unsigned char ReceivedDataBuffer[64];
        #pragma udata HID_CUSTOM_IN_DATA_BUFFER = HID_CUSTOM_IN_DATA_BUFFER_ADDRESS
        unsigned char ToSendDataBuffer[64];
        #pragma udata

    #else defined(__XC8)
        unsigned char ReceivedDataBuffer[64] @ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;
        unsigned char ToSendDataBuffer[64] @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;
    #endif
#else
    unsigned char ReceivedDataBuffer[64];
    unsigned char ToSendDataBuffer[64];
#endif

volatile USB_HANDLE USBOutHandle;    
volatile USB_HANDLE USBInHandle;

#define USB_BUFFER_SIZE     64

/** Static Functions ******************************************************/
static void JumpToBootCode(void); // Jump to Boot Code 0x001C address

/*********************************************************************
* Function: void USB_Interface_Initialize(void);
*
* Overview: Initializes the Custom HID demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void USB_Interface_Initialize()
{
    uint8_t x;
    //initialize the variable holding the handle for the last
    // transmission
    USBInHandle = 0;

    //enable the HID endpoint
    USBEnableEndpoint(CUSTOM_DEVICE_HID_EP, USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = (volatile USB_HANDLE)HIDRxPacket(CUSTOM_DEVICE_HID_EP,(uint8_t*)&ReceivedDataBuffer,USB_BUFFER_SIZE);
    
    for (x = 0; x < USB_BUFFER_SIZE; x++)
    {
        ReceivedDataBuffer[x] = 0;
        ToSendDataBuffer[x] = 0;
    }
}

void USB_Interface_100ms_Poll(void)
{

}


void USB_Interface_Fast_Poll(void)
{
    /* If the USB device isn't configured yet, we can't really do anything
     * else since we don't have a host to talk to.  So jump back to the
     * top of the while loop. */
    if( USBGetDeviceState() < CONFIGURED_STATE )
    {
        return;
    }

    /* If we are currently suspended, then we need to see if we need to
     * issue a remote wakeup.  In either case, we shouldn't process any
     * keyboard commands since we aren't currently communicating to the host
     * thus just continue back to the start of the while loop. */
    if( USBIsDeviceSuspended()== true )
    {
        return;
    }
    
    //Check if we have received an OUT data packet from the host
    if(HIDRxHandleBusy(USBOutHandle) == false)
    {   
        //We just received a packet of data from the USB host.
        //Check the first uint8_t of the packet to see what command the host
        //application software wants us to fulfill.
        switch(ReceivedDataBuffer[0])				//Look at the data the host sent, to see what kind of application specific command it sent.
        {

                // Special Emulator Command to Lock / Unlock the drawer
            case CMD_EMU_SET_DRAWER_LOCK_STATUS:
                //ucDrawerState = ReceivedDataBuffer[1]; // - I think this was relevent in a previous version. 
                break;

                // Special Emulator command
            case CMD_EMU_SET_DRAWER_OPEN_STATUS:
                break;

                // Special Emulator Command to Change the state of the tools
            case CMD_EMU_SET_TOOL_STATUS:
                break;

                // VERSION COMMAND
            case CMD_CHECK_VERSION:
                {
                    uint8_t SendBuffer[3];
                    SendBuffer[0] = CMD_CHECK_VERSION;
                    SendBuffer[1] = FirmwareVersionNumber[0];/*VERSION_MAJOR;*/
                    SendBuffer[2] = FirmwareVersionNumber[1]; /*VERSION_MINOR; */
                    Send_USB_Msg(SendBuffer, 3);
                }
                break;

                // LDR STATUS PACKET
            case CMD_CHECK_TOOL:
                GetkabTRAKLDRStatus(ReceivedDataBuffer[1]/*DrawID*/, ReceivedDataBuffer[2]/*LDR Number */);
                break;

                // TURN BUZZER ON
            case CMD_BUZZER_ON:
            case CMD_BUZZER_OFF:
                break;

                // LIGHT COMMAND & OTHER COMMANDS
            case CMD_LIGHT_DRAWER:
            case CMD_DRAWER_GREEN_LED :
            case CMD_DRAWER_RED_LED :
            case CMD_DRAWER_LOCK :
                TurnLightPanel(ReceivedDataBuffer[1]/*Draw ID*/, ReceivedDataBuffer[2] /* Light State */);
                break;

                // Get current switch status
            case CMD_USER_SWITCH :
                {
                    uint8_t SendBuffer[4];
                    SendBuffer[0] = CMD_USER_SWITCH;
                    SendBuffer[1] = !GetSwitchState(Switch2);
                    SendBuffer[2] = !GetSwitchState(Switch3);
                    Send_USB_Msg(SendBuffer, 4);
                }
                break;

                // LOCK DRAWERS
            case CMD_LOCK_CABINET_DRAWERS:
                Lock_Draws();
                break;

                // UNLOCK DRAWERS
            case CMD_UNLOCK_CABINET_DRAWERS:
                Unlock_Draws();
                break;

                // Return Drawer Open/Close status
            case CMD_CHECK_DRAWER_OPEN:
                Check_Draw_Open();
                break;

            // UPDATED FOR VERSION 2.01 TO SHOW ACTUAL CONDITION
            // 0  = UNLOCKED; 1 = LOCKED; 0xFF = ERROR
            case CMD_CHECK_DRAWER_LOCK:
                //ucDrawerNumber                  = ReceivedDataBuffer[1];
                {
                    uint8_t SendBuffer[2];
                    SendBuffer[0] = CMD_CHECK_DRAWER_LOCK;
                    SendBuffer[1] = Lock_Status();
                    /* Translate the status to what the Tablet is expecting.. */
                    if (SendBuffer[1] == Locked || SendBuffer[1] == Locking)
                    {
                        /* The unit is locked.*/
                        SendBuffer[1] = 1;
                    }
                    else {
                        /* The unit is unlocked.*/
                        SendBuffer[1] = 0;
                    }
                    
                    Send_USB_Msg(SendBuffer, 2);
                }
                break;
                // jump to boot code if you get a command FE
            case CMD_DISCONNECT_BATTERY:
                Disconnect_Battery();
                break;
            case 0xFE:
                USBOutHandle = HIDRxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ReceivedDataBuffer, USB_BUFFER_SIZE);
                //mLED_ORANGE_On();
                Blocking_Delay_ms(250); // Delay of 10000*250 Cycles (messured at about 208.20ms)

                //mLED_ORANGE_Off();
                Blocking_Delay_ms(250); // Delay of 10000*250 Cycles (messured at about 208.20ms)
                
                /* disable interrupts */
                while (INTCON2bits.GIE == 1) INTCON2bits.GIE = 0;
                JumpToBootCode();
                break;

        }


        //Re-arm the OUT endpoint, so we can receive the next OUT data packet 
        //that the host may try to send us.
        USBOutHandle = HIDRxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ReceivedDataBuffer, USB_BUFFER_SIZE);
    }

}


void Send_USB_Msg(uint8_t *Data, uint8_t Length)
{
    /* Allow for a small delay if the USB isn't ready.. */
    uint8_t Retries = 3;
    
    while (Retries > 0 && HIDTxHandleBusy(USBInHandle))
    {
        Retries--;
        Blocking_Delay_ms(10);
    }
    
    if (!HIDTxHandleBusy(USBInHandle)) {
        memcpy((void*)ToSendDataBuffer, (void*)Data, Length);
        
        //Prepare the USB module to send the data packet to the host
        USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ToSendDataBuffer[0],USB_BUFFER_SIZE);
    }
    
    
}


static void JumpToBootCode(void) 
{
    asm("goto 0x001C");
}



