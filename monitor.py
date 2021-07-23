# coding=utf-8
import sys
import pygame
from naoqi import ALProxy

IP_Blue = "192.168.1.107"                                                                                                                               
IP_Red = "192.168.1.106"
PORT = 9559


def status(ip, port):
    units = ["HeadPitch", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw",
             "RHand", "RHipYawPitch", "RHipPitch", "RHipRoll", "RKneePitch",
             "RAnklePitch", "RAnkleRoll", "HeadYaw", "LShoulderRoll", "LShoulderPitch", "LElbowYaw",
             "LElbowRoll", "LWristYaw", "LHand", "LHipYawPitch", "LHipPitch",
             "LHipRoll", "LKneePitch", "LAnklePitch", "LAnkleRoll"]

    memory = ALProxy("ALMemory", ip, port)

    pygame.init()
    size = 700, 466
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption(ip)
    background = pygame.image.load('pic.png')
    background = pygame.transform.scale(background, (695, 455))
    background_rect = background.get_rect()
    font = pygame.font.SysFont('arial', 16)

    values = ["Device/SubDeviceList/%s/Temperature/Sensor/Value",
              "Device/SubDeviceList/%s/Hardness/Actuator/Value",
              "Device/SubDeviceList/%s/ElectricCurrent/Sensor/Value",
              "Device/SubDeviceList/%s/Position/Actuator/Value",
              "Device/SubDeviceList/%s/Position/Sensor/Value",
              "Device/SubDeviceList/%s/Temperature/Sensor/Status"]
    key = ["Temperature", "Hardness", "ElectricCurrent", "PositionActuator", "PositionSensor", "TemperatureStatus"]
    select = 0
    value = values[select]
    running = True
    refresh = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    select += 1
                    select %= len(values)
                if event.button == 3:
                    select -= 1
                    if select < 0:
                        select = len(values) - 1
                value = values[select]
        screen.fill([255, 255, 255])
        screen.blit(background, background_rect)

        text = font.render("IP %s" % ip, False, (0, 0, 0))
        screen.blit(text, (5, 0))
        text2 = font.render(key[select], False, (0, 0, 0))
        screen.blit(text2, (120, 0))
        if select == 0:
            data = memory.getData("Device/SubDeviceList/Head/Temperature/Sensor/Value")
            text = font.render("CPU %.1fC" % data, False, (0, 0, 0))
            screen.blit(text, (200, 30))
            data = memory.getData("Device/SubDeviceList/Battery/Temperature/Sensor/Value")
            text = font.render("Battery %.1fC" % data, False, (0, 0, 0))
            screen.blit(text, (220, 185))
        elif select == 2:
            data = memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
            text = font.render("Battery %.1f%%" % (data * 100), False, (0, 0, 0))
            screen.blit(text, (220, 170))
            data = memory.getData("Device/SubDeviceList/Battery/Current/Sensor/Value")
            text = font.render("Battery %.1fmA" % (data * 1000), False, (0, 0, 0))
            screen.blit(text, (220, 200))
        x, y, i = 5, 30, 0
        for unit in units:
            colour = 0, 0, 0
            if i == 7:
                data = 0
            else:
                data = memory.getData(value % unit)
            if select == 0 and data >= 60.0:
                colour = 205, 175, 0
                if data >= 80.0:
                    colour = 255, 0, 0
            if select == 1 and data >= 0.9:
                colour = 205, 175, 0
            if select == 2 and data >= 0.5:
                colour = 205, 175, 0
                if data > 1:
                    colour = 255, 0, 0
            if select == 5 and data >= 1:
                colour = 205, 175, 0
                if data >= 3:
                    colour = 255, 0, 0
            string = ""
            if select == 0:
                string += unit + " %.1fC" % data
            elif select == 1:
                string += unit + " %.1f%%" % (data * 100)
            elif select == 2:
                string += unit + " %.1fmA" % (data * 1000)
            elif select == 3 or select == 4:
                string += unit + " %.2frad" % data
            elif select == 5:
                string += unit + " " + ("0-normal", "1-overheat", "2-reduce", "3-Power off")[int(data)]
            text = font.render(string, False, colour)
            screen.blit(text, (x, y))
            y += 31
            i += 1
            if i > 12 and x != 500:
                x, y = 500, 30

        pygame.display.flip()
        refresh += 1

        print "\r", refresh,
    pygame.quit()


if __name__ == '__main__':
    # status(IP_Blue, PORT)
    if len(sys.argv) > 1:
        status(sys.argv[1], PORT)
    else:
        # status(IP_Blue, PORT)
        status(IP_Red, PORT)
