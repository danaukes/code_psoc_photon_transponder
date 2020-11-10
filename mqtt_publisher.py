#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 14 13:02:13 2020

@author: danaukes
"""

# Import the MQTT library
import paho.mqtt.client as mqtt			

# The time library is useful for delays
import time					
 
# Our "on message" event
def callback(client, userdata, message):
	topic = str(message.topic)
	message = str(message.payload.decode("utf-8"))
	print(topic +': '+ message)

# Create a MQTT client object
my_client = mqtt.Client("EGR304_mqtt_client")		

# Connect to a test MQTT broker
my_client.connect("test.mosquitto.org", 1883)	

# Subscribe to the topic "EGR_304_XYZ"
# my_client.subscribe("EGR_304_XYZ")			

# Attach the messageFunction to subscription
# my_client.on_message = callback		

# Start the MQTT client
# my_client.loop_start()				
 
# Main program loop

# while(1):
    # Sleep for a second
    # time.sleep(1)