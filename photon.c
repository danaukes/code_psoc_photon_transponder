#include <MQTT.h>

#define BUFFERLEN 100

const char delimiter='\r';

const char client_name[]="EGR_304_Photon_Client_XYZ";
const char topic_publish[]="EGR304/XYZ";
const char topic_subscribe[]="EGR304/XYZ";

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

//keepalive time should be larger than rate lmit.
const int keepalive_time = 30;

//rate limit should be on the order of a second
const int rate_limit = 1;

//define the last message type
int last_type = 0;

//store the last send time
long t_reconnect = 0;

//MQTT Client Class
MQTT client("egr314.ddns.net", 1883, 30, callback);

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
    client.connect(client_name);
    //Subscribe to topic variable with MQTT client
    client.subscribe(topic_subscribe);
    //Initialize t_last_send with current time
    t_last_send = Time.now();
    //Initialize t_reconnect with current time
    t_reconnect = Time.now();
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
            
            
            if (((Time.now()-t_last_send)>rate_limit) ||  (last_type==1))
            {
                //Publish the message to the MQTT client
                client.publish(topic_publish,message_out_usb);
    
                //record the last send time as now
                t_last_send = Time.now();

                //last type was a general message
                last_type =0 ;
            }

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

            if (((Time.now()-t_last_send)>rate_limit) ||  (last_type==1))
            {
                //Publish the message to the MQTT client
                client.publish(topic_publish,message_out_rxtx);
    
                //record the last send time as now
                t_last_send = Time.now();

                //last type was a general message
                last_type =0 ;
            }
            
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
    if (client.isConnected())
    {
        if ((Time.now()-t_last_send)>keepalive_time)
        {   
            //send a keepalive message
            client.publish(topic_publish,"keepalive");
            
            //update last-sent time
            t_last_send = Time.now();
            
            //last type was a keepalive
            last_type =1 ;
        }
    }
    else
    {
        if ((Time.now()-t_reconnect)>3)
        {
            Serial.write("Client disconnected.\r\n");
            Serial1.write("Client disconnected.\r\n");
            client.disconnect();
            client.connect(client_name);
            t_reconnect = Time.now();
        }
    }

    //make sure you run the mqtt client loop
    client.loop();
}
