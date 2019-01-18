//#include <i2c.h>
#include "Timer.h"
#include "Draw_Interface.h"
#include "Cabinet_Interface.h"
#include "USB_Interface.h"
#include <xc.h>
#include <i2c.h>
#include "Board_LEDs.h"

#define NUMBER_OF_DRAWERS       10
#define LDR_COUNT 20
#define PACKET_CHECKSUM_OFFSET  20
#define TOTAL_PACKET_SIZE       21


typedef enum {
    I2C_Err_Bus_Collision = 0,
    I2C_Err_Not_Ack,
    I2C_Err_Write_Collision,
    I2C_Err_TimeOut,
    I2C_Err_Max_ID
} I2C_Error_t;


/** INITIAL STATES *******************************************************/
#define DRAWER_STATUS_UNKNOWN                       0xFF

/* These IDs are to help us keep track of the draw's operation */
#define DRAW_STATUS_ID_ACTIVE                   0x01
#define DRAW_STATUS_STATE_CHANGED               0x02
#define DRAW_STATUS_LDR_CHANGED                 0x04

/* These are the status's produced by the MUX boards and are stored in the first slot when details come back from the board.*/
#define DRAWER_STATUS_CLOSED                            0x00
#define DRAWER_STATUS_OPEN                              0x01
#define DRAWER_STATUS_LIGHT                             0x10
#define DRAWER_STATUS_SLAVE_PRESENT                     0x20
#define DRAWER_STATUS_SLAVE_COMMS                       0x40


#define I2C_TIMEOUT_COUNT   0x80

/* This stores the error counts for debugging purposes. */
static uint8_t Draw_Error_Cnt[NUMBER_OF_DRAWERS][I2C_Err_Max_ID];
/* Stores our internal status of the draws (eg if they're online etc) */
static uint8_t sDraw_Status[NUMBER_OF_DRAWERS];
/* Stores the Mux's returned details */
static uint8_t sDraw_Mux_Details[NUMBER_OF_DRAWERS][LDR_COUNT];
/* We poll each draw in turn in seperate poll calls. This variable lets us know where we're up to between calls. */
static uint8_t CurrentDrawCheckID = 0xFF;


static unsigned char GetControlAddress(unsigned char ucDrawNumber);
static unsigned char CalculateChecksum(unsigned char *ucData, unsigned char ucLength);
static uint8_t Draw_Mux_Read(uint8_t DrawID, uint8_t *pReadData, uint8_t ReadLength);
static uint8_t Draw_Mux_Write(uint8_t DrawID, uint8_t *pWriteData, uint8_t WriteLength);
static void Discover_Active_Draws(void);
static void Send_Draw_Details_To_Tablet(uint8_t Cmd_ID, uint8_t DrawID);
static uint8_t Store_Draw_Details(uint8_t DrawID, uint8_t *ucI2CReadArray);
static uint8_t Get_Details_From_Draw(uint8_t DrawID, uint8_t *ucDataArray);


void Draw_Interface_Init(void)
{
    uint8_t DrawID, Err_ID;
    unsigned int config2, config1;
    
    for (DrawID = 0; DrawID < NUMBER_OF_DRAWERS; DrawID++) {
        sDraw_Status[DrawID] = 0; /* Clear the status of all draws */
        
        /* And init the error counters to zero. */
        for(Err_ID = 0; Err_ID < I2C_Err_Max_ID; Err_ID++)
        {
            Draw_Error_Cnt[DrawID][Err_ID] = 0;
        }
    }
    
    
    /* To talk to the Draws (well the mux boards attached to them) we use I2C, so initalise the interface here.. */
    
    /* Setup the I2C Bus to the Slave device. */
    /* Baud rate is set for 100 kHz */
    config2 = 0x172;                 // was 0x11
    
    /* Configure I2C for 7 bit address mode */
    config1 = (I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
             I2C1_IPMI_DIS & I2C1_7BIT_ADD &
             I2C1_SLW_DIS & I2C1_SM_DIS &
             I2C1_GCALL_DIS & I2C1_STR_EN &                    //I2C2_STR_DIS
             I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
             I2C1_STOP_DIS & I2C1_RESTART_DIS &
             I2C1_START_DIS);
    
    OpenI2C1(config1,config2);
    
    Blocking_Delay_ms(1000);
    
    /* Find out what draws are active in the cabinet */
    Discover_Active_Draws();
}

void Draw_Interface_Fast_Poll(void)
{
    /* If we've not finished polling all the draws, request the details from the next draw. */
    if (CurrentDrawCheckID < NUMBER_OF_DRAWERS)
    {
        /* If the draw ID is valid, poll it */
        if (1 /* sDraw_Status[CurrentDrawCheckID] & DRAW_STATUS_ID_ACTIVE */ )
        {
            uint8_t ucI2CReadArray[25];

            if (Get_Details_From_Draw(CurrentDrawCheckID, ucI2CReadArray) == 0)
            {
                sDraw_Status[CurrentDrawCheckID] |= DRAW_STATUS_ID_ACTIVE;
                
                /* Got a good response, so check if the draw's status has changed. */
                if (Store_Draw_Details(CurrentDrawCheckID, ucI2CReadArray))
                {
                    /* The details have been modified, so send them to the Tablet.*/
                    Send_Draw_Details_To_Tablet(CMD_CHECK_DRAWER_FULL, CurrentDrawCheckID);
                }
            }
            else {
                /* Didn't get any details, so mark the draw as inactive. */
                sDraw_Status[CurrentDrawCheckID] &= ~DRAW_STATUS_ID_ACTIVE;
            }
        }
        CurrentDrawCheckID++;
    }
}

void Draw_Interface_100ms_Poll(void)
{
    /* If the system is not currently polling the Draws, start it going again. */
    if (CurrentDrawCheckID >= NUMBER_OF_DRAWERS)
    {
        CurrentDrawCheckID = 0;
    }
}


static uint8_t Get_Details_From_Draw(uint8_t DrawID, uint8_t *ucDataArray)
{
    uint8_t ucNumberOfAttempts = 3;
    uint8_t RetVal = 0xFF;
    
            
    while ( RetVal != 0 && ucNumberOfAttempts > 0)
    {
        ucNumberOfAttempts--;
        
        RetVal = Draw_Mux_Read(DrawID, ucDataArray, TOTAL_PACKET_SIZE);

        if (RetVal == 0)
        {
            uint8_t ucChecksum;
            // if we get here, everything has gone well, so we should check the
            // checksum for errors

            // version 2.01 changed from 14 to 20
            ucChecksum = CalculateChecksum(ucDataArray, (TOTAL_PACKET_SIZE-1));

            // version 2.01 changed from 14 to 20
            if ( ucChecksum != ucDataArray[PACKET_CHECKSUM_OFFSET])
            {
                RetVal = -5;
            }
        }
        else {
            /* Denote the error in our counts */
            Draw_Error_Cnt[DrawID][((-1*RetVal)-1)]++;
            Blocking_Delay_ms(2); /* We failed to get the I2C data, so add a 2ms delay before we go again. */
        }
        
        
    }
    return ( RetVal );
}


static void Send_Draw_Details_To_Tablet(uint8_t Cmd_ID, uint8_t DrawID)
{
    uint8_t SendBuffer[22];
    uint8_t LDR_Cnt;
    
    SendBuffer[0] = Cmd_ID;
    SendBuffer[1] = DrawID + 1;
    for (LDR_Cnt = 0; LDR_Cnt < LDR_COUNT; LDR_Cnt++) {
        SendBuffer[LDR_Cnt+2] = sDraw_Mux_Details[DrawID][LDR_Cnt];
    }
    
    // Clear and change flags from the draw.
    sDraw_Status[DrawID] &= ~(DRAW_STATUS_STATE_CHANGED|DRAW_STATUS_LDR_CHANGED);
    
    Send_USB_Msg(SendBuffer, 22);
    //Delay10KTCYx(5); // Small delay (about 4.164ms)
}

static unsigned char CalculateChecksum(unsigned char *ucData, unsigned char ucLength)
{
    unsigned char ucChecksum = 0;

    while (ucLength > 0)
    {
        ucChecksum += *ucData;
        ucData++;
        ucLength--;
    }

    ucChecksum = -ucChecksum;
    return ucChecksum;
}

/******************************************************************************
 *
 * GetControlAddress - Returns the control address given the drawer number
 *
 ******************************************************************************/
static unsigned char GetControlAddress(unsigned char ucDrawNumber) {
    return (0xA0 + (ucDrawNumber * 2)); // return the address
}

/******************************************************************************
 * TURNLIGHTPANEL - Turns light panel on or off
 * ****************************************************************************/
unsigned char TurnLightPanel(unsigned char ucDrawerNumber, unsigned char ucStatus) {

    unsigned char ucResponse;
    
    Blocking_Delay_ms(2); /* Wait for 2ms to ensure the I2C bus has enough time to settle since the last command. */
    ucResponse = Draw_Mux_Write(ucDrawerNumber, &ucStatus, 1);
    if (ucResponse != 0) {
        uint8_t SendBuffer[2];
        SendBuffer[0] = 0xAA; // Error
        SendBuffer[1] = ucResponse;
        Send_USB_Msg(SendBuffer, 2);
    }
    // return the response
    return ucResponse;
}

/* Search the I2C address space to find attached draws. */
static void Discover_Active_Draws(void)
{
    uint8_t DrawID;
    uint8_t ucI2CReadArray[TOTAL_PACKET_SIZE];
    
    for (DrawID = 0; DrawID < NUMBER_OF_DRAWERS; DrawID++)
    {
        if (Get_Details_From_Draw(DrawID, ucI2CReadArray) == 0)
        {
            /* Got a good response, so mark the fact and store the draw's initial state */
            sDraw_Status[DrawID] |= DRAW_STATUS_ID_ACTIVE;
            Store_Draw_Details(DrawID, ucI2CReadArray);
            /* We don't care about changes, so ignore the return value.*/
            sDraw_Status[DrawID] &= ~(DRAW_STATUS_STATE_CHANGED | DRAW_STATUS_LDR_CHANGED);
        }
        else {
            
            sDraw_Status[DrawID] &= ~DRAW_STATUS_ID_ACTIVE;
        }
    }
}

static uint8_t Store_Draw_Details(uint8_t DrawID, uint8_t *ucI2CReadArray)
{
    uint8_t RetVal = 0;
    uint8_t LDR_Cnt;
    
    // Check the previous state and compare it with the current state
    if (sDraw_Mux_Details[DrawID][0] != ucI2CReadArray[0])
    {
        //mLED_RED_Toggle();
        RetVal = DRAW_STATUS_STATE_CHANGED;
        sDraw_Status[DrawID] |= DRAW_STATUS_STATE_CHANGED;
        sDraw_Mux_Details[DrawID][0] = ucI2CReadArray[0];
    }
    
    // now we need to check the LDR for this drawer and see whether any
    // change has been noted.
    for (LDR_Cnt = 1; LDR_Cnt < LDR_COUNT; LDR_Cnt++) {
        if (sDraw_Mux_Details[DrawID][LDR_Cnt] != ucI2CReadArray[LDR_Cnt])
        {
            //mLED_ORANGE_Toggle();
            sDraw_Mux_Details[DrawID][LDR_Cnt] = ucI2CReadArray[LDR_Cnt];
            RetVal |= DRAW_STATUS_LDR_CHANGED;
            sDraw_Status[DrawID] |= DRAW_STATUS_LDR_CHANGED; /* Not sure if we need this.. */
        }
    }
    
    return RetVal;
}

void Check_Draw_Open(void)
{
    uint8_t DrawID;
    uint8_t SendBuffer[NUMBER_OF_DRAWERS+1];
    SendBuffer[0] = CMD_CHECK_DRAWER_OPEN;
    for (DrawID = 0; DrawID < NUMBER_OF_DRAWERS; DrawID++) {
        if (sDraw_Status[DrawID] & DRAW_STATUS_ID_ACTIVE) {
            SendBuffer[DrawID + 1] = sDraw_Mux_Details[DrawID][0]&DRAWER_STATUS_OPEN;
        } else {
            SendBuffer[DrawID + 1] = 0xFF;
        }
    }
    Send_USB_Msg(SendBuffer, NUMBER_OF_DRAWERS+1);
}
/*******************************************************************************
 *
 * GetkabTRAKLDRStatus - This function fills the array with the tool status,
 *                        returns an error if it fails during i2c communications
 *
 *****************************************************************************/
void GetkabTRAKLDRStatus(uint8_t DrawID, uint8_t LDR_Number) {
    /* The command is a request from the Tablet, which means it's DrawID is 1-10, not 0-9 like we do internally, so modify it..*/
    DrawID--;
    Send_Draw_Details_To_Tablet(CMD_CHECK_TOOL, DrawID);
}
 
 
static uint8_t Draw_Mux_Read (uint8_t DrawID, uint8_t *pReadData, uint8_t ReadLength)
{
    uint8_t TimeOutCnt = I2C_TIMEOUT_COUNT;
    
    Set_LED_On(Yellow_LED);
    
    /* make sure the requested draw ID is valid */
    if (DrawID >= NUMBER_OF_DRAWERS)
    {
        /* The Draw ID isn't valid. */
        return ( 1 );
    }
    
    I2C1STATbits.BCL = 0;       //ec- clear bus col flag
    I2C1STATbits.IWCOL = 0;       //ec- clear  wr col flag

    IdleI2C1();                              // ensure module is idle
    StartI2C1();                             // initiate START condition
    
    TimeOutCnt = I2C_TIMEOUT_COUNT;
    while (TimeOutCnt > 0 && I2C1CONbits.SEN ) TimeOutCnt--;              // wait until start condition is over
    if (TimeOutCnt == 0){ return -10; }
    
    if ( I2C1STATbits.BCL )                   // test for bus collision
    {
        Set_LED_Off(Yellow_LED);
        return ( -1 );                      // return with Bus Collision error
    }
    else
    {
         if ( MasterWriteI2C1( GetControlAddress(DrawID+1) | 0x01 ) == -1 ) // Write address plus lowest bit set for a Read operation.
        {
            StopI2C1();                      // Send stop condition
            
            Set_LED_Off(Yellow_LED);
            return ( -3 );                  // set error for write collision
        }

        IdleI2C1();                          // ensure module is idle
        if ( !I2C1STATbits.ACKSTAT )         // test for ACK condition, if received
        {
            if ( MastergetsI2C1( ReadLength, pReadData, 200) )// read in multiple bytes  // version 2.01 changed from 15 to 21
            {
                Set_LED_Off(Yellow_LED);
                return ( -1 );              // return with Bus Collision error
            }
             
            NotAckI2C1();                    // send not ACK condition
            TimeOutCnt = I2C_TIMEOUT_COUNT;
            while (TimeOutCnt > 0 &&  I2C1CONbits.ACKEN ) TimeOutCnt--;    // wait until ACK sequence is over
            if (TimeOutCnt == 0){ return -10; }
            
            StopI2C1();                      // send STOP condition
            TimeOutCnt = I2C_TIMEOUT_COUNT;
            while (TimeOutCnt > 0 &&  I2C1CONbits.PEN ) TimeOutCnt--;      // wait until stop condition is over
            if (TimeOutCnt == 0){ return -10; }
            
            if ( I2C1STATbits.BCL )           // test for bus collision
            {
                Set_LED_Off(Yellow_LED);
                return ( -1 );              // return with Bus Collision error
            }
        }
        else
        {
            StopI2C1();                      // Send Stop Condition
            Set_LED_Off(Yellow_LED);
            return ( -2 );                  // return with Not Ack error
        }
    }

    Set_LED_Off(Red_LED);
    return ( 0 );                           // return with no error
}
 
static uint8_t Draw_Mux_Write(uint8_t DrawID, uint8_t *pWriteData, uint8_t WriteLength)
{
    Set_LED_On(Yellow_LED);
        
    /* make sure the requested draw ID is valid */
    if (DrawID > NUMBER_OF_DRAWERS)
    {
        /* The Draw ID isn't valid. */
        Set_LED_Off(Yellow_LED);
        return ( 1 );
    }
    
    IdleI2C1();                          // ensure module is idle
    StartI2C1();                         // initiate START condition
     while(I2C1CONbits.SEN);          // wait until start condition is over
    if ( I2C1STATbits.BCL )               // test for bus collision
    {
        Set_LED_Off(Yellow_LED);
        return ( -1 );                    // return with Bus Collision error
    }
    else                                // start condition successful
    {
        MasterWriteI2C1( GetControlAddress(DrawID) & 0xFE );      // write byte - R/W bit should be 0
        
        IdleI2C1();                      // ensure module is idle
        if ( !I2C1STATbits.ACKSTAT )     // test for ACK condition, if received
        {
            while (WriteLength > 0)
            {
                if ( MasterWriteI2C1( *pWriteData ) )   // data byte
                {
                    StopI2C1();  // Send stop condition
                    Set_LED_Off(Yellow_LED);
                    return ( -1 * (I2C_Err_Write_Collision+1) );          // set error for write collision
                }
                pWriteData++;
                WriteLength--;
            }
        }
        else
        {
            StopI2C1();                  // Send Stop Condition
            Set_LED_Off(Yellow_LED);
            return ( -2 );              // return with Not Ack error condition
        }
    }

    IdleI2C1();                           // ensure module is idle
    StopI2C1();                           // send STOP condition
    while ( I2C1CONbits.PEN );           // wait until stop condition is over
    if ( I2C1STATbits.BCL )                // test for bus collision
    {
        Set_LED_Off(Yellow_LED);
        return ( -1 );                     // return with Bus Collision error
    }

    Set_LED_Off(Yellow_LED);
    return ( 0 );                        // return with no error
 }
