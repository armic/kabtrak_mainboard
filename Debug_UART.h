/* 
 * File:   Debug_UART.h
 * Author: rfisher
 *
 * Created on 13 February 2017, 10:48 AM
 */

#ifndef DEBUG_UART_H
#define	DEBUG_UART_H

#ifdef	__cplusplus
extern "C" {
#endif

void Init_Debug_UART(void);

void Debug_Print(char* Msg);
void Debug_Printf(char* Msg, ...);

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_UART_H */

