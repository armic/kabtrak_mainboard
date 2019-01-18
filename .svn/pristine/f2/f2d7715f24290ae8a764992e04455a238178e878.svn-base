/*******************************************************************************
  I2C funtions
  
  Company:
    Microchip Technology Inc.

  File Name:
  i2c_slavedrv.c

  Summary:
    This file is used to configure I2C.

  Description:
    This code example shows how to use I2C module in slave mode.
 The master I2C device uses the slave I2C device as RAM.
 Thus master I2C device can read/write particular RAM area of I2C slave device.
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
#include <xc.h>
#include <i2c.h>
#include <stdint.h>
#include "I2C_Interface.h"
#define USE_I2C_Clock_Stretch

#define DIP1            PORTAbits.RA2       // Dip switch 1
#define DIP2            PORTAbits.RA3       // Dip switch 2
#define DIP3            PORTAbits.RA4       // Dip switch 3
#define DIP4            PORTAbits.RA5       // Dip switch 4

/*****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************/
//uint8_t         ramBuffer[256]; //RAM area which will work as EEPROM for Master I2C device
//uint8_t         *ramPtr;        //Pointer to RAM memory locations
uint8_t         Incomming_Packet[21];



//uint16_t    count = 0;
//uint16_t    count2 = 0;

//char        data1 = 0;
//char        command = 0;        // command for i2c
//char        comd_to_send = 0;   // flag

static int8_t I2C_Active = 0;
static int8_t Command_To_Execute = 0xFF;
static uint8_t *pOutgoingPacket = 0;


// *****************************************************************************
// Section: Function definition
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
 * Function:       void I2C1_Init(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes I2C1 peripheral.
 *****************************************************************************/

void I2C1_Init( void )
{
    #if !defined( USE_I2C_Clock_Stretch )
    I2C1CON = 0x8000;       //Enable I2C1 module
    #else
    I2C1CON = 0x9040;       //Enable I2C1 module, enable clock stretching
    #endif

    // 7 bit address for MUX boards is defined as follows:
    // MSB   -    LSB
    // 1,0,1,A,B,C,D,R/W
    //
    //where ABCD are values of S1 inputs
    //   A=RA5,  B=RA4,  C=RA3,     D=RA2
    //  bits 3,2 and 1 are fixed 101 respectively
    //  R/W = Read / Write command bit
    //

    uint8_t address = 0x50; //adds bits 4,6 & 6 as 101 for leading address 
    
    if(!DIP4)       // if set - sets bit 3 of I2C address
        address |= 0x08;
    
    if(!DIP3)       // if set - sets bit 2 of I2C address
        address |= 0x04;

    if(!DIP2)       // if set -  sets bit 1 of I2C address
        address |= 0x02;

    if(!DIP1)       // if set -  sets bit 0 of I2C address
        address |= 0x01;
    
    // bits to set in range of A0 to BF

    I2C1ADD = address;      //assign address to I2C address register

    IFS1 = 0;

    _SI2C1IF = 0;
    _SI2C1IE = 1;
}

int8_t Get_Command_To_Execute(void)
{
    int8_t RetVal = Command_To_Execute;
    Command_To_Execute = 0xFF;
    
    return RetVal;
}

void Set_Outgoing_Packet(uint8_t * pData)
{
    //_SI2C1IE = 0;
    pOutgoingPacket = pData;
    //_SI2C1IE = 1;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This is the ISR for I2C1 Slave interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void )
{
    static uint8_t Data_Loc = 0;
    if( (I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0) )    //Address matched
    {
        unsigned char   temp;   //used for dummy read
        temp = I2C1RCV;     //dummy read
        Data_Loc = 0; /* Reset our sending position. */
#if defined( USE_I2C_Clock_Stretch )
        I2C1CONbits.SCLREL = 1;                 //Release SCL1 line
#endif

    }
    else if( (I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1) )   //check for data
    {
        I2C_Active = 20;
        Command_To_Execute = I2C1RCV;          // reads in command
        LED_D2 ^= 1;
#if defined( USE_I2C_Clock_Stretch )
        I2C1CONbits.SCLREL = 1;                 //Release SCL1 line
#endif
    }
    else if( (I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0) )
    {
        unsigned char   temp;   //used for dummy read
        I2C_Active = 20;
        temp = I2C1RCV;
        
        if (pOutgoingPacket == 0)
        {
            uint8_t x;
            for (x=0; x<21; x++)
            {
                I2C1TRN = 0;

                I2C1CONbits.SCLREL = 1; //Release SCL1 line
                while( I2C1STATbits.TBF );
            }
        }
        else {
            uint8_t x;
            //pOutgoingPacket[20] = CalcChecksum(pOutgoingPacket, 20);
            
            for (x=0; x<21; x++)
            {
                I2C1TRN = pOutgoingPacket[x];

                I2C1CONbits.SCLREL = 1; //Release SCL1 line
                while( I2C1STATbits.TBF );
            }
        }
    }

    _SI2C1IF = 0;               //clear I2C1 Slave interrupt flag
}

uint8_t CalcChecksum(uint8_t *pData, uint8_t Data_Length)
{
    uint8_t RetVal = 0;
    
    while (Data_Length > 0)
    {
        RetVal += *pData;
        pData++;
        Data_Length--;
    }
    
    RetVal = -RetVal; //2s complement (neg of itself)
    
    return RetVal;
}

uint8_t Is_I2C_Active(void)
{
    if (I2C_Active > 0)
    {
        --I2C_Active;
        return 1;
    }
    
    return 0;
}

/*******************************************************************************
 End of File
*/
