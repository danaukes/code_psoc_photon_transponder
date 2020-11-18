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
                //don't process rxdata buffer
                read_val = 0;
                //clear the rxdata buffer
                clear_buf(rxdata,100);
                //reset the ii counter
                ii=0;
            }
            //if you get terminating characters, stop reading into your rx buffer and process the incoming number.
            else if(c==';' || c=='.')
            {
                //process rxdata buffer
                read_val = 1;
                //reset the ii counter
                ii=0;
                break;
            }
            //otherwise, add the character to your rxdata buffer and increment your ii counter.
            else
            {
                //don't process rxdata buffer
                read_val = 0;
                //save c into rxdata buffer
                rxdata[ii]=c;
                //increment the ii counter
                ii++;
            }
        }
        //check if read_val value is nonzero
        if (read_val!=0)
        {
            //read the first character of the rxdata buffer.  If it is an l, process it as a long int.
            if (rxdata[0]=='l')
            {
                //clear out the next character in rxdata to ensure it's treated as an end of line.
                rxdata[ii]=0;
                //convert string in rxdata to a long int, starting with the first character after the 0th position
                lval = strtol( rxdata+1, &ptr, 10 );    // convert message to an integer
                //clear the rxdata buffer
                clear_buf(rxdata,100);
                //reset the ii counter
                ii=0;
                
            }
            
        }

        //reset the read_val variable
        read_val = 0;
        
        //turn LED on if lval is greater than 50.
        if (lval>50)
        {
            Pin_1_Write(1);
        }
        //turn LED off if lval is less than 50.
        else
        {
            Pin_1_Write(0);
        }
        
        //clear the txdata buffer
        //clear_buf(txdata,100);
        //write lval to txdata
        l=sprintf( txdata, "lval = %ld\r\n", lval);


        //only update every 100 cycles
        if (kk==100)
        {
            //wait until USB system is ready
            while (!myUSB_CDCIsReady());
            //write txdata to USB
            myUSB_PutString(txdata);
            //reset kk counter
            kk=0;
        }
        //increment kk counter
        kk++;

    }
}

/* [] END OF FILE */
