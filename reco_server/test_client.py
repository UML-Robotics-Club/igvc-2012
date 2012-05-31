#!/usr/bin/env python
import socket
import time
import sys

if len(sys.argv) < 4:
    sys.exit('Usage: ./test_client.py HOST "command" times')

HOST = sys.argv[1]
PORT = 9042

command = sys.argv[2]
repeat  = int(sys.argv[3])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(data):
    sock.sendto(data + "\n", (HOST, PORT))

def recv():
    return sock.recv(1024)

if __name__ == '__main__':
    print "HI"
    send("0;HI")

    for ii in range(repeat):
        ticket = recv()
        print "ticket:", ticket

        send("%s;%s\n"  % (ticket, command))
        print "%04d: %s;%s\n" % (ii, ticket, command)
