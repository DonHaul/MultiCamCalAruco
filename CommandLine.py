import threading
import cv2
import Queue as queue
import datetime
import time

import open3d

from libs import *

import rospy

def worker(invoker,stop):

    x=""

    while True:

        x= raw_input("Enter command")

        

        commands = x.split()

        if len(commands)>0:
            invoker.execute(commands[0],commands[1:])


        if stop():
            break




"""
Encapsulate a request as an object, thereby letting you parameterize
clients with different requests, queue or log requests, and support
undoable operations.
"""

import abc
import time


class Invoker:
    """
    Ask the command to carry out the request.
    """

    def __init__(self):
            self._commands = {}
            self._history = []

    @property
    def history(self):
        return self._history

    def register(self, command_name, command):
        self._commands[command_name] = command

    def execute(self, command_name,*args):
        if command_name in self._commands.keys():
            self._history.append((time.time(), command_name,args))
            self._commands[command_name].execute(args)
        else:
            print("Command  not recognised")





class Receiver:
    """
    Know how to perform the operations associated with carrying out a
    request. Any class may serve as a Receiver.
    """

    def help(self,*args):
        print("No one is here to help you")

    def help1(self,*args):
        print("No one is here to help you",args)

    def help2(self,*args):
        print("No one is here to help you",args[0],args[1])

t1=[]

def Start(state,commandImporter):
    print("Starting Commandline")

    invoker = Invoker()
    commandImporter(invoker,state)


    t1 = threading.Thread(target=worker,args=(invoker,state.GetStop,))
    t1.start()

def Stop():
    t1.join() 