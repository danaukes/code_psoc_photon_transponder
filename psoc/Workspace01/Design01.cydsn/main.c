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

//internal variable
char c;

//pointer to USB serial data
uint8 pdata[64];

//length variable
uint16 len;

int main(void)
{
    //the following code gets run once
    
    //enable global interrupts
    CyGlobalIntEnable;

    //Start the USB subsystem
    UART_photon_Start();
  
    //Start the USB Serial Peripheral
    myUSB_Start(0,myUSB_5V_OPERATION);
    
    //wait until the USB configuration is loaded
    while(0 == myUSB_GetConfiguration())
    {
    }
    //prepare the USB Serial System for receiving data
    myUSB_CDC_Init();
    
    //this is like a while loop
    for(;;)
    {

        //if there are characters in the UART receive buffer
        if (UART_photon_GetRxBufferSize()>0)
        {
            //retrieve one character and save in c
            c = UART_photon_GetChar();
            //wait until USB is ready to send
            while (!myUSB_CDCIsReady());
            //put character in
            myUSB_PutChar(c);
        }
        
        //if there is data in the USB receive buffer`
        if (myUSB_DataIsReady())
        {
            //read how many bytes are available
            len = myUSB_GetCount();
            
            //if there are more than 64 bytes, restrict to first 64
            if (len>64) len=64;
            
            //read data and return the actual number of bytes read to len
            len = myUSB_GetData(pdata,len);
            
            //iterate through all received bytes 
            for (int jj=0;jj<len;jj++)
            {
                //send one character at a time into UART 
                UART_photon_PutChar(pdata[jj]);
            }
        }

    }
}

/* [] END OF FILE */
