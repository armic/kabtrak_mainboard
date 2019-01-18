/* 
 * File:   Timer.h
 * Author: rfisher
 *
 * Created on 27 July 2016, 1:05 PM
 */

#ifndef TIMER_H
#define	TIMER_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
    
void Init_Timer1( void );
uint16_t Get_Timer_Count(void);
void Blocking_Delay_ms(uint16_t DelayTime);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_H */

