#include <MQTT.h>

#define BUFFERLEN 100

const char delimiter='\r';

const char topic[50]="EGR_304_XYZ";

//counter variable for message_out array
int ii_usb = 0;

//usb-based UART message_out array
char message_out_usb[BUFFERLEN];

//counter variable for message_out array
int ii_rxtx = 0;

//Rx/Tx-pin-based UART message_out array
char message_out_rxtx[BUFFERLEN];

//internal variable
char c=0;

//store the last send time
long t_last_send = 0;

//MQTT Client Class
MQTT client("test.mosquitto.org", 1883, 30, callback);

//declare the callback that fires when a subscribed topic comes in
void callback(char* topic, byte* payload, unsigned int length) 
{
    /*
    //Extra Debugging text
    Serial.write("Message received on topic: ");
    Serial.write(topic);
    Serial.write("\r\n");
    */

    //iterate through each character in the incoming payload
    for (int jj=0;jj<length;jj++) 
    {
        //write each character individually to USB
        Serial.write((char)payload[jj]);
    }
    //write a newline
    Serial.write("\r\n");

    /*
    //Extra Debugging text
    Serial1.write("Message received on topic: ");
    Serial1.write(topic);
    Serial1.write("\r\n");
    */

    //iterate through each character in the incoming payload
    for (int jj=0;jj<length;jj++) 
    {
        //write each character individually to Tx
        Serial1.write((char)payload[jj]);
    }
    //write a newline
    Serial1.write("\r\n");

}

void setup() {
    //Open USB-Serial at 9600 baud
    Serial.begin(9600);
    //Opwn Rx/Tx pins at 9600 baud
    Serial1.begin(9600);
    //Set up MQTT client for publishing on topic variable
    client.connect(topic);
    //Subscribe to topic variable with MQTT client
    client.subscribe(topic);
    //Initialize t_last_send with current time
    t_last_send = Time.now();
}

void loop() {
    
    //while characters are available on USB-Serial
    while (Serial.available()>0)
    {   
        //read one character, store as c
        c = Serial.read();
        
        //Write c back out to usb-serial
        //Serial.write(c);
        
        //if c is the end-of-line delimiter:
        if (c==delimiter)
        {
            //write a zero to the message(terminating the string)
            message_out_usb[ii_usb]=0;

            //Write the message out to USB-Serial
            //Serial.write(message_out_usb);
            //Serial.write("\r\n");
            
            //Publish the message to the MQTT client
            client.publish(topic,message_out_usb);

            //record the last send time as now
            t_last_send = Time.now();
            
            //reset my internal counter
            ii_usb=0;
            
            //leave the while loop
            break;
        }
        //otherwise
        else
        {
            //add c to the usb message buffer
            message_out_usb[ii_usb]=c;

            //increment the usb message buffer counter
            ii_usb++;        

            //if the usb message buffer counter is at the limit, reset it to zero, truncating the message
            if (ii_usb==BUFFERLEN) ii_usb=0;
        }
        
    }

    //while characters are available on RxTx-Serial
    while (Serial1.available()>0)
    {   
        //read one character, store as c
        c = Serial1.read();

        //Write c back out to Rx/Tx Serial
        //Serial1.write(c);

        //if c is the end-of-line delimiter:
        if (c==delimiter)
        {
            //write a zero to the message(terminating the string)
            message_out_rxtx[ii_rxtx]=0;

            //Write the message out to USB-Serial
            //Serial1.write(message_out_rxtx);
            //Serial1.write("\r\n");

            //Publish the message to the MQTT client
            client.publish(topic,message_out_rxtx);

            //record the last send time as now
            t_last_send = Time.now();

            //reset my internal counter
            ii_rxtx=0;

            //leave the while loop
            break;
        }
        //otherwise
        else
        {
            //add c to the usb message buffer
            message_out_rxtx[ii_rxtx]=c;

            //increment the usb message buffer counter
            ii_rxtx++;        

            //if the usb message buffer counter is at the limit, reset it to zero, truncating the message
            if (ii_rxtx==BUFFERLEN) ii_rxtx=0;
        }
    }

    //if the current time is greater than 10 seconds since last send,
    if ((Time.now()-t_last_send)>10)
    {   
        //send a keepalive message
        client.publish(topic,"keepalive");
        
        //update last-sent time
        t_last_send = Time.now();
    }

    //make sure you run the mqtt client loop
    client.loop();
}