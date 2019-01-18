
#include "Timer.h"
#include <xc.h>
#include "Board_LEDs.h"

static uint16_t Timer_Count = 0;

/******************************************************************************
 * Function:       void Init_Timer1( void )
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       Initialize Timer1 for Period Interrupts
 *****************************************************************************/
void Init_Timer1( void )
{
    T1CON = 0;          // Timer reset
    //T1CONbits.
    IFS0bits.T1IF = 0;  // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 6;  // Timer1 Interrupt priority level=4
    
    TMR1 = 0x0000;
    PR1 = 60010;       // Timer1 period register = 0xEA6B which gives us a 1ms interrupt period
    T1CONbits.TON = 1;  // Enable Timer1 and start the counter
    IEC0bits.T1IE = 1;  // Enable Timer1 interrupt
}



/******************************************************************************
 * Function:       void __attribute__((interrupt,no_auto_psv)) _T1Interrupt( void )
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       ISR ROUTINE FOR THE TIMER1 INTERRUPT
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _T1Interrupt( void )
{
    
    if (Timer_Count == 0xFFFF)
    {
        Timer_Count = 0;
    }
    else {
        Timer_Count++;
    }
    
    IFS0bits.T1IF = 0; /* reset Timer 1 interrupt flag */
    
}

uint16_t Get_Timer_Count(void)
{
    return Timer_Count;
}

void Blocking_Delay_ms(uint16_t DelayTime)
{
    uint16_t Wakeup_Value = DelayTime + Get_Timer_Count();
    
    while (Get_Timer_Count() != Wakeup_Value);
    
}