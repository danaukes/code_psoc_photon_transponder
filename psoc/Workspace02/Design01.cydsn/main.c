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
#include <stdlib.h>
#include "project.h"

//internal variable
char c;

//pointer to USB serial data
char rxdata[100];
char txdata[100];

//declare some counters
int ii=0;
int kk=0;

//declare pointer for use in conversion
char *ptr;

//
int read_val = 0;

long lval = 0;

//length variable
uint16 len;

int l=0;

void clear_buf(char * buffer,int len)
{
    for (int ii=0;ii<len;ii++)
    {
        buffer[ii]=0;
    }
}

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
        while(UART_photon_GetRxBufferSize()>0)
        {
            //retrieve one character and save in c
            c = UART_photon_GetChar();

            //make a decision based on the character received
                //if you get newline characters, reset the counter, you are going to receive a new message
            if(c=='\n' || c=='\r')
            {
                read_val = 0;
                clear_buf(rxdata,100);
                ii=0;
            }
            else if(c==';' || c=='.')
            {
                read_val = 1;
                ii=0;
                break;
            }
            else
            {
                read_val = 0;
                rxdata[ii]=c;
                ii++;
            }
        }
        
        if (read_val!=0)
        {
            if (rxdata[0]=='l')
            {
                rxdata[ii]=0;
                lval = strtol( rxdata+1, &ptr, 10 );    // convert message to an integer
                clear_buf(rxdata,100);
                ii=0;
                
            }
            
        }

        read_val = 0;
        
        if (lval>50)
        {
            Pin_1_Write(1);
        }
        else
        {
            Pin_1_Write(0);
        }
        
        //clear_buf(txdata,100);
        l=sprintf( txdata, "Hello! %ld\r\n", lval);
                
        while (!myUSB_CDCIsReady());
            //put character in
        if (kk==100)
        {
            myUSB_PutString(txdata);
            kk=0;
        }
        kk++;

    }
}

/* [] END OF FILE */
