import pygame
from pygame.locals import *
import sys
import socket

def grab(x, y, w, h):
    "Grab a part of the screen"
    # get the dimension of the surface
    rect = pygame.Rect(x, y, w, h)
    # copy the part of the screen
    sub = screen.subsurface(rect)
    # create another surface with dimensions
    # This is done to unlock the screen surface
    # Unlock screen surface here:
    screenshot = pygame.Surface((w, h))
    screenshot.blit(sub, (0, 0))
    return screenshot

## Here we define the UDP IP address as well as the port number that we have
## already defined in the client python script.
UDP_IP_ADDRESS = "0.0.0.0"
UDP_PORT_NO = 6789


pygame.init()

flags = DOUBLEBUF
size = width, height = 1000, 1000
# speed = [2, 2]
black = 0, 0, 0

screen = pygame.display.set_mode(size, flags)

WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

pos = 0
trail = []
trail_pos = 0
track_points = []

pos = 0

serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
## One difference is that we will have to bind our declared IP address
## and port number to our newly declared serverSock
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))
first = True

font = pygame.font.SysFont(None, 24)
maxspeed = 0

# offset x is 300 and offset y is 350
right_starting_point = (236.0460891723633, 296.31824493408203)
left_starting_point = (244.4570121765137, 302.9487609863281)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: 
            sys.exit()
    
    data, addr = serverSock.recvfrom(1024)
    data_array = data.decode().split(";")
    for elem in data_array:
        element_parts = elem.split(",")
        if len(element_parts) == 4:
            position = float(element_parts[1])+300, float(element_parts[2])+350
            print(position)
            speed = float(element_parts[3])
            # print(speed)
            if speed > maxspeed:
                maxspeed = speed
            simid = element_parts[0]

            if first and pos < 21:
                track_points.append(position)
                pos+=1
                trail.append(position)
                trail_pos+=1
                # first = True
            else:
                if first:
                    pos = len(track_points)-2
                    trail_pos = len(trail)-2
                    first = False

                screen.fill(black)
                track_points.append(position)
                if trail_pos >= 20:
                    trail_pos = 0
                else:
                    trail_pos += 1
                trail[trail_pos]=position
                for i in range(pos):
                    # print(i)
                    pygame.draw.circle(screen, BLUE, track_points[i], 1)
                for i in range(20):
                    if i == trail_pos:
                        pygame.draw.circle(screen, RED, trail[i], 5)
                    else:
                        pygame.draw.circle(screen, WHITE, trail[i], 5)
                pos += 1
                img = font.render('simid: '+simid+", speed: "+str(int(speed))+", maxspeed: "+str(int(maxspeed)), True, WHITE)
                screen.blit(img, (20, 20))

                sub = grab(0, 40, 950, 950)
                surf = pygame.transform.rotate(sub, 90)
                surf = pygame.transform.flip(surf, False, True)
                screen.blit(surf, (0, 40))
        pygame.display.flip()
