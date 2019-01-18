#include "Board_LEDs.h"
#include <xc.h>
#include <stdint.h>


    /* LED_GRN - RJ5 */
    /* LED_YEL - RJ6 */
    /* LED_RED - RJ7 */
    /* 7SEG_A - RJ10 */
    /* 7SEG_B - RJ11 */
    /* 7SEG_C - RJ13 */
    /* 7SEG_D - RJ14 */
    /* 7SEG_E - RJ15 */
    /* 7SEG_F - RJ9 */
    /* 7SEG_G - RJ8 */
    /* 7SEG_DP - RJ12 */

#define LED_GRN LATJbits.LATJ5
#define LED_YEL LATJbits.LATJ6
#define LED_RED LATJbits.LATJ7

#define _7SEG_A LATJbits.LATJ10
#define _7SEG_B LATJbits.LATJ11
#define _7SEG_C LATJbits.LATJ13
#define _7SEG_D LATJbits.LATJ14
#define _7SEG_E LATJbits.LATJ15
#define _7SEG_F LATJbits.LATJ9
#define _7SEG_G LATJbits.LATJ8
#define _7SEG_DP LATJbits.LATJ12

#define _7SEG_A_Bit (0x0001<<10)
#define _7SEG_B_Bit (0x0001<<11)
#define _7SEG_C_Bit (0x0001<<13)
#define _7SEG_D_Bit (0x0001<<14)
#define _7SEG_E_Bit (0x0001<<15)
#define _7SEG_F_Bit (0x0001<<9)
#define _7SEG_G_Bit (0x0001<<8)

#define _7SEG_MASK  ( _7SEG_A_Bit | _7SEG_B_Bit | _7SEG_C_Bit | _7SEG_D_Bit | _7SEG_E_Bit | _7SEG_F_Bit | _7SEG_G_Bit)

static uint8_t Current_7Seg_Value;

void Init_Board_LEDs(void)
{
    Current_7Seg_Value = 0xFF; /* Mark as cleared. */
    
    TRISJ &= /* LED_GRN */      ~((0x01<<5) |
             /* LED_YEL*/       (0x01<<6) |
             /* LED_RED*/       (0x01<<7) |
             /* 7SEG_A*/        (0x01<<10) |
             /* 7SEG_B*/        (0x01<<11) |
             /* 7SEG_C*/        (0x01<<13) |
             /* 7SEG_D*/        (0x01<<14) |
             /* 7SEG_E*/        (0x01<<15) |
             /* 7SEG_F*/        (0x01<<9) |
             /* 7SEG_G*/        (0x01<<8) |
             /* 7SEG_DP*/       (0x01<<12));
}

void Set_LED_On(LEDs_t LED)
{
    switch (LED)
    {
        case Red_LED:
            LED_RED = 1;
            break;
        case  Yellow_LED:
            LED_YEL = 1;
            break;
        case Green_LED:
            LED_GRN = 1;
            break;
        case DotPoint_LED:
            _7SEG_DP = 1;
            break;
    }
}
void Set_LED_Off(LEDs_t LED)
{
    switch (LED)
    {
        case Red_LED:
            LED_RED = 0;
            break;
        case  Yellow_LED:
            LED_YEL = 0;
            break;
        case Green_LED:
            LED_GRN = 0;
            break;
        case DotPoint_LED:
            _7SEG_DP = 0;
            break;
    }
}
void Toggle_LED(LEDs_t LED)
{
    switch (LED)
    {
        case Red_LED:
            LED_RED ^= 1;
            break;
        case  Yellow_LED:
            LED_YEL ^= 1;
            break;
        case Green_LED:
            LED_GRN ^= 1;
            break;
        case DotPoint_LED:
            _7SEG_DP ^= 1;
            break;
    }
}

static void Clear_7Segment(void)
{
    /*
    _7SEG_A = 0;
    _7SEG_B = 0;
    _7SEG_C = 0;
    _7SEG_D = 0;
    _7SEG_E = 0;
    _7SEG_F = 0;
    _7SEG_G = 0;
      */
    LATJ &= ~_7SEG_MASK;
    //LATJ = 0;
}

static const uint16_t Char_Array[10] = {
    /*0*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_C_Bit | _7SEG_D_Bit | _7SEG_E_Bit | _7SEG_F_Bit), 
    /*1*/ (_7SEG_B_Bit | _7SEG_C_Bit), 
    /*2*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_G_Bit | _7SEG_E_Bit | _7SEG_D_Bit),
    /*3*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_G_Bit | _7SEG_C_Bit | _7SEG_D_Bit), 
    /*4*/ (_7SEG_F_Bit | _7SEG_G_Bit | _7SEG_B_Bit | _7SEG_C_Bit), 
    /*5*/ (_7SEG_A_Bit | _7SEG_F_Bit | _7SEG_G_Bit | _7SEG_C_Bit | _7SEG_D_Bit), 
    /*6*/ (_7SEG_F_Bit | _7SEG_E_Bit | _7SEG_D_Bit | _7SEG_C_Bit | _7SEG_G_Bit), 
    /*7*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_C_Bit),
    /*8*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_C_Bit | _7SEG_D_Bit | _7SEG_E_Bit | _7SEG_F_Bit | _7SEG_G_Bit), 
    /*9*/ (_7SEG_A_Bit | _7SEG_B_Bit | _7SEG_C_Bit | _7SEG_F_Bit | _7SEG_G_Bit) 
};

void Set_7Segment_Char(char Chr)
{
    Clear_7Segment();
    
    if ((Chr >= '0') && (Chr <= '9'))
    {
        LATJ |= Char_Array[Chr - '0'];
        Current_7Seg_Value = (Chr - '0');
    }
    else {
        Current_7Seg_Value = 0xFF;
    }
}

/* Loop that does a figure of 8 */
static const uint16_t Loop1_Array[8] = {
    _7SEG_A_Bit,
    _7SEG_B_Bit,
    _7SEG_G_Bit,
    _7SEG_E_Bit,
    _7SEG_D_Bit,
    _7SEG_C_Bit,
    _7SEG_G_Bit,
    _7SEG_F_Bit
};

/* Loop that does a circle */
static const uint16_t Loop2_Array[8] = {
    _7SEG_A_Bit,
    _7SEG_B_Bit,
    _7SEG_C_Bit,
    _7SEG_D_Bit,
    _7SEG_E_Bit,
    _7SEG_F_Bit
};

void Step_7Segment_Loop1(void)
{
    Clear_7Segment();
    
    if ((Current_7Seg_Value >= 0xF0) && (Current_7Seg_Value < 0xF7))
    {
        Current_7Seg_Value++;
    }
    else {
        Current_7Seg_Value = 0xF0;
    }
    
    LATJ |= Loop1_Array[Current_7Seg_Value - 0xF0];
    
}

void Step_7Segment_Loop2(void)
{
    Clear_7Segment();
    
    if ((Current_7Seg_Value >= 0xE0) && (Current_7Seg_Value < 0xE5))
    {
        Current_7Seg_Value++;
    }
    else {
        Current_7Seg_Value = 0xE0;
    }
    
    LATJ |= Loop2_Array[Current_7Seg_Value - 0xE0];
    
}
