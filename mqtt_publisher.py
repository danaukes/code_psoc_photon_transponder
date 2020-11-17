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
 
# # Our "on message" event
# def callback(client, userdata, message):
# 	topic = str(message.topic)
# 	message = str(message.payload.decode("utf-8"))
# 	print(topic +': '+ message)

# Create a MQTT client object
my_client = mqtt.Client("EGR_304_Python_Publisher_DMA")		

# Connect to a test MQTT broker
my_client.connect("embedded-systems.ddns.net", 1883)	

while(1):
    # Sleep for a second
    time.sleep(1)
    a=my_client.publish('EGR_304_DMA','this is a test')
    a.wait_for_publish()