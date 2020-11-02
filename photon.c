#include <MQTT.h>
#define BUFFERLEN 100

const char delimiter='\r';

int ii_usb = 0;
char message_out_usb[BUFFERLEN];

int ii_rxtx = 0;
char message_out_rxtx[BUFFERLEN];

char c=0;
long t_last_send = 0;

MQTT client("test.mosquitto.org", 1883, 30, callback);

void callback(char* topic, byte* payload, unsigned int length) 
{
    //Serial.write("Message received on topic: ");
    //Serial.write(topic);
    //Serial.write("\r\n");
    for (int jj=0;jj<length;jj++) 
    {
        Serial.write((char)payload[jj]);
    }
    Serial.write("\r\n");

    //Serial1.write("Message received on topic: ");
    //Serial1.write(topic);
    //Serial1.write("\r\n");
    for (int jj=0;jj<length;jj++) 
    {
        Serial1.write((char)payload[jj]);
    }
    Serial1.write("\r\n");

}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    client.connect("EGR_304_DMA");
    client.subscribe("EGR_304_DMA");
    t_last_send = Time.now();
}

void loop() {
    
    while (Serial.available()>0)
    {   
        c = Serial.read();
        //Serial.write(c);
        if (c==delimiter)
        {
            message_out_usb[ii_usb]=0;
            //Serial.write(message_out_usb);
            //Serial.write("\r\n");
            client.publish("EGR_304_DMA",message_out_usb);
            t_last_send = Time.now();
            ii_usb=0;
            break;
        }
        else
        {
            message_out_usb[ii_usb]=c;
            ii_usb++;        
            if (ii_usb==BUFFERLEN) ii_usb=0;
        }
        
    }

    while (Serial1.available()>0)
    {   
        c = Serial1.read();
        //Serial1.write(c);
        if (c==delimiter)
        {
            message_out_rxtx[ii_rxtx]=0;
            //Serial1.write(message_out_rxtx);
            //Serial1.write("\r\n");
            client.publish("EGR_304_DMA",message_out_rxtx);
            t_last_send = Time.now();
            ii_rxtx=0;
            break;
        }
        else
        {
            message_out_rxtx[ii_rxtx]=c;
            ii_rxtx++;        
            if (ii_rxtx==BUFFERLEN) ii_rxtx=0;
        }
        
    }


    if ((Time.now()-t_last_send)>10)
    {
        client.publish("EGR_304_DMA","keepalive");
        t_last_send = Time.now();
    }

    client.loop();

}