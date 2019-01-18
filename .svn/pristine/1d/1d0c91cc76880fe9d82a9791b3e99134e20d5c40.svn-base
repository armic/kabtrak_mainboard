/*******************************************************************************
  ce445 I2C API header file

  Company:
    Microchip Technology Inc.

  File Name:
    i2c_slavedrv.h

  Summary:
    I2C API function definitions.

  Description:
    This header file consists of the declaration of the I2C initialization function and the I2C ISR. 
 Also Buffer for storing data and flag stucture.
*******************************************************************************/
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <i2c.h>

//#define __dsPIC33E__

#ifdef __cplusplus  // Provide C++ Compatability
extern "C"
{
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void    I2C1_Init( void );
uint8_t CalcChecksum(uint8_t* pData, uint8_t Data_Length);

int8_t Get_Command_To_Execute(void);
void Set_Outgoing_Packet(uint8_t * pData);
uint8_t Is_I2C_Active(void);

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define ON              1
#define OFF             0

//Slave MUX constants

#define LEDS_OFF        0x00
#define LEDS_ON         0x01
#define GRN_LED_OFF     0x02
#define GRN_LED_ON      0x03
#define RED_LED_OFF     0x04
#define RED_LED_ON      0x05
#define LOCK_OFF        0x06
#define LOCK_ON         0x07

#define LED_D2          LATAbits.LATA14      // chk RED led
#define DRW_GRN         LATHbits.LATH5      // chk RED led
#define DRW_RED         LATHbits.LATH4      // chk RED led
#define DRW_LOCK        LATDbits.LATD11      // MOD from LDR in 1
#define Light_Panel     LATAbits.LATA15     // chk drive FET to light LED panel

#define I2C_PACKET_SIZE 21




/*******************************************************************************
 End of File
*/
