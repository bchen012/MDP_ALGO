#!/usr/bin/env python3
""" Handling communication with RaspeberryPi

Msg format:   
"""
import socket
import os
import time
from algo.constants import PORT, FORMAT, ADDR, DISCONNECT_MESSAGE

class RPi:
    def __init__(self):
        # create a TCP/IP socket
        print(f'[START] establishing a connection...')
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(ADDR)
        print(f'[CONNECT] connect to RPi on {ADDR}')

    def send(self, msg):
        message = msg.encode(FORMAT)
        self.client.send(message)
        print(f'[SEND] {message}')
    
    def receive(self):
        msg = self.client.recv(2048).decode(FORMAT)
        if msg:
            print(f'[RECEIVED] {msg}')
        return msg

    def disconnect(self):
        self.send(DISCONNECT_MESSAGE)
        self.client
        

if __name__ == '__main__':
    rpi = RPi()
    while True:
        received = rpi.receive()
        if received:
            rpi.send('Hi from laptop')
            break
    rpi.disconnect()
        





