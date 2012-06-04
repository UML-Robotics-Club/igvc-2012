#!/bin/bash

SSID="stark-teleop"
WLAN=wlan1

if [ "$USER" != "root" ]; 
then
    sudo $0;
    exit 0;
fi

nmcli dev disconnect iface $WLAN
ifconfig $WLAN down
ifconfig $WLAN up
iwconfig $WLAN mode master 
iwconfig $WLAN channel 4
iwconfig $WLAN essid $SSID
ifconfig $WLAN 10.0.0.1 up
