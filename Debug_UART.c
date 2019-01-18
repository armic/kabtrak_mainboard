#include "Debug_UART.h"
#include <stdint.h>
#include <uart.h>
#include "Timer.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define FCY               60000000        /* Instruction Cycle Frequency 60MHz */
#define BAUDRATE          115200          /* Our desired baudrate. */
#define BRGVAL          (((FCY/BAUDRATE)/4)-1)

#define TMP_BUFF_SIZE 250

static char TmpBuff[TMP_BUFF_SIZE];

static char TxBuffer[TMP_BUFF_SIZE];
static uint16_t Tx_Insert_Loc;
static uint16_t Tx_Remove_Loc;

void Init_Debug_UART(void)
{

    RPINR18bits.U1RXR = 69; /* Rx On ping RP69/RD5 */
    RPOR3bits.RP70R = 0x01; /* Tx on pin RP70/RD6 */
    
    U1MODEbits.STSEL = 1; // 2-stop bits
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 1; // High Speed mode
    U1MODEbits.RTSMD = 1;
    U1BRG = BRGVAL; // BAUD Rate Setting
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 0; // Disable UART Tx interrupt
    IEC0bits.U1RXIE = 1; // Enable UART Rx interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
    Blocking_Delay_ms(1);
    
    Tx_Insert_Loc = 0;
    Tx_Remove_Loc = 0;
}


void Debug_Print(char* Msg)
{
    uint16_t Msg_Length = strlen(Msg);
    
    while (Msg_Length >0)
    {
        TxBuffer[Tx_Insert_Loc] = *Msg;
        Msg++;
        
        if (++Tx_Insert_Loc >= TMP_BUFF_SIZE)
        {
            Tx_Insert_Loc = 0;
        }
        
        Msg_Length--;
        
    }
    
    /* We've just added data to be sent out, so make sure the interrupt is enabled and triggered to start sending data out. */
    IEC0bits.U1TXIE = 1;
    IFS0bits.U1TXIF = 1;
    
}


void Debug_Printf(char* Msg, ...)
{
  va_list variable_arguments;

  va_start(variable_arguments, Msg);
  
  vsprintf(TmpBuff, Msg, variable_arguments);
  
  Debug_Print(TmpBuff);

  va_end(variable_arguments);
}


/* This is UART1 transmit ISR */
void __attribute__ ( (interrupt, no_auto_psv) ) _U1TXInterrupt(void)
{  
   IFS0bits.U1TXIF = 0;
   
   if (Tx_Insert_Loc != Tx_Remove_Loc)
   {
        while (Tx_Insert_Loc != Tx_Remove_Loc && U1STAbits.UTXBF != 1)
        {
            U1TXREG = TxBuffer[Tx_Remove_Loc];
            
            if (++Tx_Remove_Loc >= TMP_BUFF_SIZE)
            {
                Tx_Remove_Loc = 0;
            }
        }
   }
   else {
       /* disable the Interrupt, since nothing left to send.*/
       IEC0bits.U1TXIE = 0;
   }
} 
/* This is UART1 receive ISR */
void __attribute__ ( (interrupt, no_auto_psv) ) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    ReadUART1();
    
    
#if 0
/* Read the receive buffer till atleast one or more character can be read */ 
    while( DataRdyUART1())
      {
        ( *( Receiveddata)++) = ReadUART1();
    }
#endif
} 