/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

//Include standard libraries
#include <math.h>
#include <stdio.h>
#include "project.h"

char c;

uint8 pdata[64];
uint16 len;

int main(void)
{
    //the following code gets run once
    
    //enable global interrupts
    CyGlobalIntEnable;
    //Start the USB subsystem
    UART_photon_Start();
    myUSB_Start(0,myUSB_5V_OPERATION);
    while(0 == myUSB_GetConfiguration())
    {
    }
    myUSB_CDC_Init();
    

    for(;;)
    {
        //wait until myUSB subsystem is ready
        //write message to USB

        
        if (UART_photon_GetRxBufferSize()>0)
        {
            c = UART_photon_GetChar();
            while (!myUSB_CDCIsReady());
            myUSB_PutChar(c);
        }
        if (myUSB_DataIsReady())
        {
            len = myUSB_GetCount();
            if (len>64) len=64;
            len = myUSB_GetData(pdata,len);
            for (int jj=0;jj<len;jj++)
            {
                //while (!myUSB_CDCIsReady());
                //myUSB_PutChar(pdata[jj]);
                UART_photon_PutChar(pdata[jj]);
            }
        }

    }
}

/* [] END OF FILE */
