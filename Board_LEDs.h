/* 
 * File:   Board_LEDs.h
 * Author: rfisher
 *
 * Created on 7 February 2017, 12:19 PM
 */

#ifndef BOARD_LEDS_H
#define	BOARD_LEDS_H

typedef enum {
    Red_LED,
    Yellow_LED,
    Green_LED,
    DotPoint_LED
} LEDs_t;

#ifdef	__cplusplus
extern "C" {
#endif

void Init_Board_LEDs(void);

void Set_LED_On(LEDs_t LED);
void Set_LED_Off(LEDs_t LED);
void Toggle_LED(LEDs_t LED);

void Set_7Segment_Char(char Chr);
void Step_7Segment_Loop1(void);
void Step_7Segment_Loop2(void);

#ifdef	__cplusplus
}
#endif

#endif	/* BOARD_LEDS_H */

