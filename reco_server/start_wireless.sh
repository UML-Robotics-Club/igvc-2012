#!/bin/bash

WLAN=wlan0

if [ "$USER" != "root" ]; 
then
    sudo $0;
    exit 0;
fi

nmcli dev disconnect iface $WLAN
udhcpd ./udhcpd.conf &
hostapd ./hostapd.conf &
ifconfig wlan0 192.168.71.10 up
