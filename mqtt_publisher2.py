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
 
# Create a MQTT client object
my_client = mqtt.Client("EGR_304_Python_Publisher_DMA")		

# Connect to a test MQTT broker
my_client.connect("idealab.ddns.net", 1883)	

while(True):
    # Sleep for a second
    s = input('Enter Value and hit Enter: ')
    a=my_client.publish('EGR_304_DMA',s)
    a.wait_for_publish()
    # time.sleep(3)
    