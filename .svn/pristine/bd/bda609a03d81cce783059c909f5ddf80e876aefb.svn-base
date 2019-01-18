#include "Cabinet_Interface.h"
#include "stdint.h"
#include "./USB/usb.h"
//#include "HardwareProfile.h"

#include "USB_Interface.h"
#include "Timer.h"
#include "Debug_UART.h"

/* These names are too general. need to change to something more explicit */
#define TURN_OFF        0x00
#define TURN_ON         0x01

/* Analogue Inputs */
    /* u15V_Rail - RE2 (AN26)*/
    /* uVBattery - RE3 (AN27)*/
    /* uSystem_Current - RE4 (AN28)*/
    /* uTemperature - RB12 (AN12) */
    
    /* Outputs used */
    /* LOGO_LIGHT     - RE1 */
    /* Charge_Enable  - RB4 */
    /* Battery_Enable - RB5 */
    /* Lock_Direction - RG12 */
    /* Lock_Solenoid  - RG13 */
    /* SpareIO_1      - RJ0 */
    /* SpareIO_2      - RJ1 */
    
    /* Inputs used */
    /* Switch2     - RG0 */
    /* Switch3     - RG1 */
    /* Lock_Status - RG14 */

/** SWITCH *********************************************************/
#define SWITCH2_IO_PIN                 PORTGbits.RG0
#define SWITCH3_IO_PIN                 PORTGbits.RG1

/** Locking Mechanism ********************************************/
#define LOCK_STATUS         PORTGbits.RG14
#define LOCK_DIRECTION      PORTGbits.RG12
#define LOCK_SOLENOID       PORTGbits.RG13

#define LOCK_STATUS_Unlocked    1
#define LOCK_STATUS_Locked      0

#define LOCK_DIR_Unlock 0
#define LOCK_DIR_Lock   1

#define LOCK_SOLENOID_DISABLE    0 
#define LOCK_SOLENOID_ENABLE     1 

/** Spare IO Outputs ********************************************/
/* These outputs are not used at this time and are marked DNP on the
   Drawings, however I've included the code for them to make future 
   use more easy. */
#define SpareIO_1           PORTJbits.RJ0
#define SpareIO_2           PORTJbits.RJ1

/** Battery Charging and Enable Outputs*/
#define Battery_Enable      PORTBbits.RB5
#define Charge_Enable       PORTBbits.RB4

#define LOGO_LIGHT          PORTEbits.RE1

/* Variables used by the ADC sampling. */
unsigned int ADC_Temperature;
unsigned int ADC_15V_Rail;
unsigned int ADC_VBattery;
unsigned int ADC_Sys_Current;

uint32_t Accumulator_15V_Rail;
uint16_t Avg_15V_Rail;
uint32_t Accumulator_VBattery;
uint16_t Avg_VBattery;
uint16_t Accumulator_Sys_Current;
uint16_t Avg_Sys_Current;
int16_t Accumulator_Temperature;
int16_t Avg_Temperature;

struct {
    uint16_t Days;
    uint8_t Hours;
    uint8_t Mins;
    uint8_t Secs;
} UpTime;


typedef enum{
    Start_ADC_Sequence,
    Sample_ADC_Source_1,
    Sample_ADC_Source_2,
    Sample_ADC_Source_3,
    Sample_ADC_Source_4,
    ADC_Sequence_Complete
}ADC_Seq_t;

static ADC_Seq_t ADC_Seq;

typedef enum
{
    LS_Idle = 0,                    /* No action required */

    LS_Unlock_Start = 10,           /* Start unlocking the cabinet */
    LS_Unlock_Direction_Set = LS_Unlock_Start,
    LS_Unlock_Enable_Set = 11,
    /* Stay enabled for 3 * 100ms (or 0.3 second) */
    LS_Unlock_Enable_Clr = 14,
    LS_Unlock_Direction_Clr = 15,
    LS_Unlock_Finished = 16,

    LS_Lock_Start = 110,             /* Start locking the cabinet */
    LS_Lock_Direction_Set = LS_Lock_Start,
    LS_Lock_Enable_Set = 111,
    /* Stay enabled for 3 * 100ms (or 0.3 second) */
    LS_Lock_Enable_Clr = 114,
    LS_Lock_Direction_Clr = 115,
    LS_Lock_Finished = 116
} Lock_State_t;

static Lock_State_t Lock_Change_State = LS_Idle;

/*************************************************************************************/
/********************* Static Functions **********************************************/
/*************************************************************************************/
static void ReportSwitchStateChange(void);
static int16_t ADC_Counts_To_Temperature(uint16_t ADC_Counts);
static uint16_t ADC_Counts_To_15Vrail(uint16_t ADC_Counts);
static uint16_t ADC_Counts_To_VBat(uint16_t ADC_Counts);
static uint16_t ADC_Counts_to_Milli_Volts(uint16_t ADC_Counts);
static uint16_t ADC_Counts_To_Sys_Current(uint16_t ADC_Counts);

void Cabinet_Interface_Init(void)
{
    Lock_Change_State = LS_Idle;
    
    // VERSION 2.01 ADDED
    LOCK_SOLENOID    = LOCK_SOLENOID_DISABLE;   // disable lock/unlock solenoid
    LOCK_DIRECTION   = LOCK_DIR_Unlock;    // set direction to 0 (unlock)
    
    ClrWdt(); /* Kick the watchdog */
    
    /* Setup the ADC */
    ANSELBbits.ANSB12 = 1;  /* uTemp    - AN12/RB12*/
    ANSELEbits.ANSE2 = 1;   /* u15V     - AN26/RE2 */
    ANSELEbits.ANSE3 = 1;   /* uVBat    - AN27/RE3 */
    ANSELEbits.ANSE4 = 1;   /* uIsys    - AN28/RE4 */
    
    /* Initialize ADC module */
    AD1CON1 = 0x0004;
    AD1CON1bits.AD12B = 1; /* Enable 12-bit mode */
    AD1CON2 = 0x0000;
    AD1CON2bits.VCFG = 3; /* Use External VRef+ and External VRef-  */
    AD1CON3 = 0x000F;
    AD1CON4 = 0x0000;
    AD1CHS0 = 0;
    AD1CHS0bits.CH0SA = 12;
    AD1CHS123 = 0x0000;
    AD1CSSH = 0x0000;
    AD1CSSL = 0x0000;

    /* Enable ADC module and provide ADC stabilization delay */
    AD1CON1bits.ADON = 1;
    ADC_Seq = Sample_ADC_Source_1;
    
    Accumulator_15V_Rail = 0;
    Avg_15V_Rail = 0;
    Accumulator_VBattery = 0;
    Avg_VBattery = 0;
    Accumulator_Sys_Current = 0;
    Avg_Sys_Current = 0;
    Accumulator_Temperature = 0;
    Avg_Temperature = 0;
    
    /* Enable the battery to power the unit. */
    Battery_Enable = 1;
    
    UpTime.Days = 0;
    UpTime.Hours = 0;
    UpTime.Mins = 0;
    UpTime.Secs = 0;
    
    /* Turn the LED on. */
    LOGO_LIGHT = 1;
}


void Init_Cabinet_GPIO(void)
{
    
    
    TRISB &= /*Charge_Enable*/      ~((0x01<<4) |
             /*Battery_Enable*/       (0x01<<5));
    TRISE &= /*LOGO_LIGHT*/         ~(0x01<<1);
    TRISG &= /* Lock_Direction */   ~((0x01<<12) |
             /* Lock_Solenoid */      (0x01<<13));
    TRISJ &= /* SpareIO_1*/         ~((0x01<<0) |
             /* SpareIO_2*/           (0x01<<1));
    
    
}

void Cabinet_Interface_100ms_Poll(void)
{
    static uint8_t One_Second_Counter = 0;
    if (Lock_Change_State != LS_Idle)
    {
        switch(Lock_Change_State)
        {
            case LS_Unlock_Start:
            //case LS_Unlock_Direction_Set:
                LOCK_DIRECTION = LOCK_DIR_Unlock; 
                break;
            
            case LS_Lock_Enable_Set:
            case LS_Unlock_Enable_Set:
                LOCK_SOLENOID = LOCK_SOLENOID_ENABLE;
                break;
            
            case LS_Unlock_Enable_Clr:
            case LS_Lock_Enable_Clr:
                LOCK_SOLENOID = LOCK_SOLENOID_DISABLE;
                break;
            
            case LS_Unlock_Direction_Clr:
            case LS_Lock_Direction_Clr:
                LOCK_DIRECTION = LOCK_DIR_Unlock;
                break;
            
            case LS_Unlock_Finished:
            case LS_Lock_Finished:
                Lock_Change_State = LS_Idle;
                break;

            case LS_Lock_Start:
            //case LS_Lock_Direction_Set:
                LOCK_DIRECTION = LOCK_DIR_Lock;
                break;
            default:
                /* No action required. */
                break;
        }
        
        if (Lock_Change_State != LS_Idle)
        {
            Lock_Change_State++;
        }
        
    }
    
    /* We apply a low pass filter to the ADC Values to help smooth out their readings.. */
    Accumulator_15V_Rail = Accumulator_15V_Rail - (Accumulator_15V_Rail/10) + ADC_Counts_To_15Vrail(ADC_15V_Rail);
    Avg_15V_Rail = (Accumulator_15V_Rail/10);
    
    Accumulator_VBattery = Accumulator_VBattery - (Accumulator_VBattery/10) + ADC_Counts_To_VBat(ADC_VBattery);
    Avg_VBattery = (Accumulator_VBattery/10);
    
    Accumulator_Sys_Current = Accumulator_Sys_Current - (Accumulator_Sys_Current/10) + ADC_Counts_To_Sys_Current(ADC_Sys_Current);
    Avg_Sys_Current = (Accumulator_Sys_Current/10);
    
    Accumulator_Temperature = Accumulator_Temperature - (Accumulator_Temperature/10) + ADC_Counts_To_Temperature(ADC_Temperature);
    Avg_Temperature = (Accumulator_Temperature/10);
    
    /* set the next batch of ADC conversions going. */
    ADC_Seq = Sample_ADC_Source_1;
    
    /* once every second display the ADC Values and do a small amount of logic based upon them */
    if (++One_Second_Counter > 10)
    {
        /* Increment the Uptime value */
        if (++UpTime.Secs >= 60)
        {
            UpTime.Secs = 0;
            if (++UpTime.Mins >= 60)
            {
                UpTime.Mins = 0;
                if (++UpTime.Hours >= 24)
                {
                    UpTime.Hours = 0;
                    UpTime.Days++;
                }
            }
        }

        Debug_Printf("Uptime:%uD, %0.2uH, %0.2uM, %0.2uS Temp:%d V15:%d VBat:%d SysI:%d\n\r", UpTime.Days, UpTime.Hours, UpTime.Mins, UpTime.Secs, Avg_Temperature, Avg_15V_Rail, Avg_VBattery, Avg_Sys_Current);
        
        /* Now handle the switching in and out of the Charging Circuit. */
        if (Avg_15V_Rail > Avg_VBattery)
        {
            /* We're running on the mains, so enable the charge circuit. */
            Charge_Enable = 1;
        }
        else if (Avg_15V_Rail < Avg_VBattery)
        {
            /* We're running on the battery, so disable the charge circuit. */
            Charge_Enable = 0;
        }
        
        One_Second_Counter = 0;
    }
    
    
}

void Cabinet_Interface_Ms_Poll(void)
{
    ReportSwitchStateChange();
    
    switch(ADC_Seq)
    {
        case Start_ADC_Sequence:
            /* disable the ADC */
            AD1CON1bits.ADON = 0;
            /* Change to the new pin */
            AD1CHS0bits.CH0SA = 12;
            /* Turn the ADC back on. */
            AD1CON1bits.ADON = 1;
            ADC_Seq++;
            break;
        case Sample_ADC_Source_1:
            AD1CON1bits.SAMP = 0; // Start the conversion
            while (!AD1CON1bits.DONE); // Wait for the conversion to complete
            AD1CON1bits.DONE = 0; // Clear conversion done status bit
            ADC_Temperature = ADC1BUF0;
            
            /* disable the ADC */
            AD1CON1bits.ADON = 0;
            /* Change to the new pin */
            AD1CHS0bits.CH0SA = 26;
            /* Turn the ADC back on. */
            AD1CON1bits.ADON = 1;
            ADC_Seq++;
            break;
        case Sample_ADC_Source_2:
            AD1CON1bits.SAMP = 0; // Start the conversion
            while (!AD1CON1bits.DONE); // Wait for the conversion to complete
            AD1CON1bits.DONE = 0; // Clear conversion done status bit
            ADC_15V_Rail = ADC1BUF0;
            
            /* disable the ADC */
            AD1CON1bits.ADON = 0;
            /* Change to the new pin */
            AD1CHS0bits.CH0SA = 27;
            /* Turn the ADC back on. */
            AD1CON1bits.ADON = 1;
            ADC_Seq++;
            break;
        case Sample_ADC_Source_3:
            AD1CON1bits.SAMP = 0; // Start the conversion
            while (!AD1CON1bits.DONE); // Wait for the conversion to complete
            AD1CON1bits.DONE = 0; // Clear conversion done status bit
            ADC_VBattery = ADC1BUF0;
            
            /* disable the ADC */
            AD1CON1bits.ADON = 0;
            /* Change to the new pin */
            AD1CHS0bits.CH0SA = 28;
            /* Turn the ADC back on. */
            AD1CON1bits.ADON = 1;
            ADC_Seq++;
            break;
            
        case Sample_ADC_Source_4:
            AD1CON1bits.SAMP = 0; // Start the conversion
            while (!AD1CON1bits.DONE); // Wait for the conversion to complete
            AD1CON1bits.DONE = 0; // Clear conversion done status bit
            ADC_Sys_Current = ADC1BUF0;
            
            /* disable the ADC */
            AD1CON1bits.ADON = 0;
            /* Change to the new pin */
            AD1CHS0bits.CH0SA = 12;
            /* Turn the ADC back on. */
            AD1CON1bits.ADON = 1;
            ADC_Seq++;
            break;
        case ADC_Sequence_Complete:
        default:
            break;
    }
    
}



void Unlock_Draws(void)
{
    /* Start the state machine to Unlock the Draws */
    Lock_Change_State = LS_Unlock_Start;
}

void Lock_Draws(void)
{
    /* Start the state machine to Lock the Draws */
    Lock_Change_State = LS_Lock_Start;
}

void Disconnect_Battery(void)
{
    Battery_Enable = 0;
}


/* This functions detects changes to the buttons on the Cabinet and sends their changed status up to the tablet.
 *
 * Questions: Do we need to de-bounce these inputs?
 *            Why do we send 64 bytes up to the Tablet when we only set 2?
 * 
 */
static void ReportSwitchStateChange(void)
{
    static uint8_t PrevState_Switch2 = 0xFF;
    static uint8_t PrevState_Switch3 = 0xFF;
    
    if (PrevState_Switch2 == 0xFF)
    {
        PrevState_Switch2 = SWITCH2_IO_PIN;
        PrevState_Switch3 = SWITCH3_IO_PIN;
    }
    
    if ( PrevState_Switch3 != SWITCH3_IO_PIN )
    {
        uint8_t SendBuffer[2];
        PrevState_Switch3 = SWITCH3_IO_PIN;
        
        SendBuffer[0] = CMD_USER_SWITCH3;
        SendBuffer[1] = !PrevState_Switch3;
        Send_USB_Msg(SendBuffer, 2);
    }

    if ( PrevState_Switch2 != SWITCH2_IO_PIN )
    {
        uint8_t SendBuffer[2];
        PrevState_Switch2 = SWITCH2_IO_PIN;
        
        SendBuffer[0] = CMD_USER_SWITCH2;
        SendBuffer[1] = !PrevState_Switch2;
        Send_USB_Msg(SendBuffer, 2);
    }
}

uint8_t GetSwitchState(Switch_t SwitchID)
{
    switch(SwitchID)
    {
        case Switch2:
            return CMD_USER_SWITCH2;
            break;
        case Switch3:
            return CMD_USER_SWITCH3;
            break;
    }
    
    return 0;
}

Lock_Status_t Lock_Status(void)
{
    if (Lock_Change_State >= LS_Unlock_Start && Lock_Change_State <= LS_Unlock_Finished)
    {
        return Unlocking;
    }
    else if (Lock_Change_State >= LS_Lock_Start && Lock_Change_State <= LS_Lock_Finished)
    {
        return Locking;
    }
    else {
        if (LOCK_STATUS == LOCK_STATUS_Unlocked)
        {
            return Unlocked;
        }
        else {
            return Locked;
        }
    }
    
}

#define MAX_ADC_COUNTS  0x0FFF

static uint16_t ADC_Counts_to_Milli_Volts(uint16_t ADC_Counts)
{
    /* Convert the ADC Counts into milli volts. */
    uint32_t TmpValue = ADC_Counts;
    TmpValue *= 3000; /* Times by 1000 first, so we don't loose resolution. */
    TmpValue /= MAX_ADC_COUNTS;
    
    return (uint16_t)TmpValue;
}

static int16_t ADC_Counts_To_Temperature(uint16_t ADC_Counts)
{
    int16_t ReturnVal = (int16_t)ADC_Counts_to_Milli_Volts(ADC_Counts);
    
    ReturnVal -= 600; /* remove the offset from the value. */
    
    /* the sensor is 1degree per 10mV, so we don't convert it at this point so we have one degree accuracy.*/
    return ReturnVal;
}

static uint16_t ADC_Counts_To_15Vrail(uint16_t ADC_Counts)
{
    /* we need to do the calculation as a 32 bit number to avoid overflow issues. */
    uint32_t ReturnVal = ADC_Counts_to_Milli_Volts(ADC_Counts);
    ReturnVal *= 553;
    ReturnVal /= 100;
    return (uint16_t)ReturnVal;
    
}

static uint16_t ADC_Counts_To_VBat(uint16_t ADC_Counts)
{
    /* we need to do the calculation as a 32 bit number to avoid overflow issues. */
    uint32_t ReturnVal = ADC_Counts_to_Milli_Volts(ADC_Counts);
    ReturnVal *= 513;
    ReturnVal /= 100;
    return (uint16_t)ReturnVal;
}

static uint16_t ADC_Counts_To_Sys_Current(uint16_t ADC_Counts)
{
    /* 200mV = 1Amp */
    return ADC_Counts_to_Milli_Volts(ADC_Counts) * 5;
}