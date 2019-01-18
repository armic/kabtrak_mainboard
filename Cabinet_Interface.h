/* 
 * File:   Cabinet_Functions.h
 * Author: rfisher
 *
 * Created on 20 July 2016, 3:24 PM
 */

#ifndef CABINET_FUNCTIONS_H
#define	CABINET_FUNCTIONS_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
    Switch2,
    Switch3
}Switch_t;

typedef enum {
    Locked,     /* Cabinet is locked. */
    Locking,     /* Cabinet is in the process of locking. */
    Unlocked,   /* Cabinet is unlocked */
    Unlocking  /* Cabinet is in the process of unlocking */
    
} Lock_Status_t;

/* 
 * This function initalises all the variables and interfaces required to talk to the Cabinet hardware.
 */
void Cabinet_Interface_Init(void);

/* 
 * This function initalises the General Purpose IO pins of the microprocessor used to control the cabinet switches/devices.
 */
void Init_Cabinet_GPIO(void);

/*
 * This function needs to be called once every 100ms to ensure correct timing for the Cabinet operations.
 */
void Cabinet_Interface_100ms_Poll(void);
/*
 * This function needs to be called once every 100ms.
 */
void Cabinet_Interface_Ms_Poll(void);

void Lock_Draws(void);

void Unlock_Draws(void);

Lock_Status_t Lock_Status(void);

uint8_t GetSwitchState(Switch_t SwitchID);

void Disconnect_Battery(void);

#ifdef	__cplusplus
}
#endif

#endif	/* CABINET_FUNCTIONS_H */

