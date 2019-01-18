/* 
 * File:   USB_Commands.h
 * Author: rfisher
 *
 * Created on 20 July 2016, 5:10 PM
 */

#ifndef USB_COMMANDS_H
#define	USB_COMMANDS_H

#include "stdint.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define CMD_LOCK_CABINET_DRAWERS        0x01
#define CMD_UNLOCK_CABINET_DRAWERS      0x02
#define CMD_CHECK_DRAWER_LOCK           0x03
#define CMD_CHECK_TOOL                  0x04
#define CMD_EMU_SET_DRAWER_LOCK_STATUS  0x05
#define CMD_EMU_SET_TOOL_STATUS         0x06
#define CMD_CHECK_DRAWER_OPEN           0x07
#define CMD_EMU_SET_DRAWER_OPEN_STATUS  0x08

#define CMD_CHECK_DRAWER_FULL           0x10
#define CMD_CHECK_VERSION               0x11
#define CMD_BUZZER_ON                   0x12
#define CMD_BUZZER_OFF                  0x13
#define CMD_LIGHT_DRAWER                0x14
#define CMD_USER_SWITCH3                0x15
#define CMD_USER_SWITCH2                0x16
#define CMD_USER_SWITCH                 0x17
#define CMD_USER_SWITCH4                0x18 // new command  -- not sure what this is ??


// new commands for improved MUX
#define CMD_DRAWER_GREEN_LED            0x19
#define CMD_DRAWER_RED_LED              0x20
#define CMD_DRAWER_LOCK                 0x21
#define CMD_DISCONNECT_BATTERY          0x30 /* Tells the Main board to disconnect itself from the internal battery allowing it to reboot */


/* 
 * This function initalises all the variables and interfaces required to talk to the USB hardware.
 * Note: This is executed by the usb_events.c module when a USB connection has been configured, so
 *       should not be called else where.
 */
void USB_Interface_Initialize(void);
/*
 * This function needs to be called once every 100ms to ensure correct timing for the USB operations.
 */
void USB_Interface_100ms_Poll(void);
/*
 * This function needs to be called as often as possible. It does all the quick updates etc as required.
 */
void USB_Interface_Fast_Poll(void);

void Send_USB_Msg(uint8_t *Data, uint8_t Length);
void Init_USB_Interface(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* USB_COMMANDS_H */

