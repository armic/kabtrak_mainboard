/* 
 * File:   Draw_Communications.h
 * Author: rfisher
 *
 * Created on 20 July 2016, 2:55 PM
 */

#ifndef DRAW_INTERFACE_H
#define	DRAW_INTERFACE_H

#include "stdint.h"

#define I2C_TIMEOUT_COUNTER 0xFFFF

#ifdef	__cplusplus
extern "C" {
#endif



/* 
 * This function initalises all the variables and interfaces required to talk to the draws.
 */
void Draw_Interface_Init(void);
/*
 * This function needs to be called once every 100ms to ensure correct timing for the draw operations.
 */
void Draw_Interface_100ms_Poll(void);
/*
 * This function needs to be called as often as possible. It does all the quick updates etc as required.
 */
void Draw_Interface_Fast_Poll(void);

/* This function gets the Draw Interface to send the details of a specific draw up to the Tablet. The DrawID is
 * given as a value 1-10 */
void GetkabTRAKLDRStatus(uint8_t DrawID, uint8_t LDR_Number);

/* Turns the Draws IR leds on or off. DrawNumber is a value between 1-10 */
unsigned char TurnLightPanel(unsigned char ucDrawerNumber, unsigned char ucStatus);


void Check_Draw_Open(void);

#ifdef	__cplusplus
}
#endif

#endif	/* DRAW_INTERFACE_H */

