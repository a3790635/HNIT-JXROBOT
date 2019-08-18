# coding=utf-8
"""自定义模块"""
import sys
import time
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import threading

GolfGreeter = None
memory = None


class GolfGreeterModule(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)
        self.tts = ALProxy("ALTextToSpeech")
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("FrontTactilTouched", "GolfGreeter", "onFrontTactilTouched")
        memory.subscribeToEvent("MiddleTactilTouched", "GolfGreeter", "onMiddleTactilTouched")

    def onFrontTactilTouched(self, *_args):
        memory.unsubscribeToEvent("FrontTactilTouched", "GolfGreeter")
        # 触摸头前部 响应程序
        self.tts.say("Hello, you touch me front head")
        # end
        memory.subscribeToEvent("FrontTactilTouched", "GolfGreeter", "onFrontTactilTouched")

    def onMiddleTactilTouched(self, *_args):
        memory.unsubscribeToEvent("MiddleTactilTouched", "GolfGreeter")
        # 触摸头中部 响应程序
        self.tts.say("Hello, you touch me middle head")
        # end
        memory.subscribeToEvent("MiddleTactilTouched", "GolfGreeter", "onMiddleTactilTouched")

    def myMethod(self):
        self.tts.say("hello")


def main(robotIp):
    myBroker = ALBroker("GolfBroker", "0.0.0.0", 0, robotIp, 9559)
    global GolfGreeter
    GolfGreeter = GolfGreeterModule("GolfGreeter")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)


def MyClass(robotIp):
    # example
    mymodule = ALProxy("GolfGreeter", robotIp, 9559)
    mymodule.myMethod()


if __name__ == '__main__':
    # robotIp = "192.168.43.165"
    Ip = "127.0.0.1"
    t1 = threading.Thread(target=main(robotIp))
    t1.start()
    # t1.is_alive()
    # t2 = threading.Thread(target=MyClass(robotIp))
    # t2.start()
