#!/usr/bin/env python
import roslib; roslib.load_manifest('reco_server')
import rospy
rospy.init_node('reco_server')

SESSION_TIMEOUT = 5.0
COMMAND_TIMEOUT = 0.2

PORT = 9042

import SocketServer
import threading
import signal
import time
import sys
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix

import woah

client  = { 'addr' : None,
           'sock' : None,
           'stamp': 0.0 }

cond    = threading.Condition(threading.Lock())

def cond_signal():
    cond.acquire()
    cond.notify()
    cond.release()

cmd_vel = rospy.Publisher('/robot/cmd_vel', Twist)

def set_speeds(speed, rotsp):
    msg = Twist()
    msg.linear.x  = float(speed)
    msg.angular.z = float(rotsp)
    cmd_vel.publish(msg)

def send_tickets():
    while True:
        if time.time() - client['stamp'] > COMMAND_TIMEOUT:
            set_speeds(0.0, 0.0)

        # Wait for active session
        cond.acquire()
        while time.time() - client['stamp'] > SESSION_TIMEOUT:
            client['addr'] = None
            cond.wait()
        cond.release()

        # Send a ticket and sleep.
        ticket = time.time()
        client['sock'].sendto("%.02f" % ticket, client['addr'])

        time.sleep(0.05)

def got_command(data):
    cmd = data.split(" ")

    if cmd[0] == 'speed':
        set_speeds(cmd[1], cmd[2])
    else:
        print "Unknown command:", data

class PacketHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        if client['addr'] == None:
            print "New client:", self.client_address
            client['stamp'] = time.time()
            client['sock']  = self.request[1]
            client['addr']  = self.client_address
            cond_signal()
            return

        if self.client_address != client['addr']:
            print "Bad client:", self.client_address
            return

        client['stamp'] = time.time()
        cond_signal()

        (ticket_s, body) =  self.request[0].split(";", 1)
        ticket = float(ticket_s)
        
        if time.time() - ticket < COMMAND_TIMEOUT:
            got_command(body)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda _a, _b: os._exit(0))

    tthr = threading.Thread(target=send_tickets)
    tthr.start()

    server = SocketServer.UDPServer(("", PORT), PacketHandler)
    server.serve_forever()
