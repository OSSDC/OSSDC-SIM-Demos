import pygame
from pygame.locals import *
import sys
import socket
import time

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

def pos_relative_to_finish_line(x1, x2, y1, y2, xA, yA):
    v1 = (x2-x1, y2-y1)   # Vector 1
    v2 = (x2-xA, y2-yA)   # Vector 2
    xp = v1[0]*v2[1] - v1[1]*v2[0]  # Cross product
    # if xp > 0:
    #     print('on one side')
    # elif xp < 0:
    #     print('on the other')
    # else:
    #     print('on the same line!')
    return xp

## Here we define the UDP IP address as well as the port number that we have
## already defined in the client python script.
UDP_IP_ADDRESS = "0.0.0.0"
UDP_PORT_NO = 6789


pygame.init()
pygame.display.set_caption('OSSDC Race Control Center')

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

rel_prev_pos = None
rel_curr_pos = None
lap_count = 0
start_time = None
lap_time = 0
best_lap_time = None

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
            # print(position)
            speed = float(element_parts[3])
            # print(speed)
            if speed > maxspeed:
                maxspeed = speed
            # simid = element_parts[0]
            simid = addr[0]
            
            if start_time is None:
                start_time = time.time()

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
            
                (x1, y1) = left_starting_point
                (x2, y2) = right_starting_point
                (xA, yA) = position
                rel_curr_pos = pos_relative_to_finish_line(x1, x2, y1, y2, xA, yA)
                
                if rel_prev_pos is not None:
                    if rel_prev_pos > 0 and rel_curr_pos < 0:
                        lap_count += 1
                        lap_time = time.time() - start_time
                        if best_lap_time is None:
                            best_lap_time = lap_time
                        if lap_time < best_lap_time:
                            best_lap_time = lap_time
                        start_time = time.time()
                        rel_prev_pos = None
                    elif rel_curr_pos == 0:
                        lap_count += 1
                        lap_time = time.time() - start_time
                        if best_lap_time is None:
                            best_lap_time = lap_time
                        if lap_time < best_lap_time:
                            best_lap_time = lap_time
                        start_time = time.time()
                        rel_prev_pos = None
                    else:
                        rel_prev_pos = rel_curr_pos
                else:
                    if rel_curr_pos == 0:
                        lap_count += 1
                        lap_time = time.time() - start_time
                        if best_lap_time is None:
                            best_lap_time = lap_time
                        if lap_time < best_lap_time:
                            best_lap_time = lap_time
                        start_time = time.time()
                    else:
                        rel_prev_pos = rel_curr_pos


                pos += 1
                img = font.render('simid: '+simid+", speed: "+str(int(speed))+", max speed: "+str(int(maxspeed))+", lap: "+str(lap_count)+", laptime: "+str(lap_time)+", fastest lap: "+str(best_lap_time), True, WHITE)
                screen.blit(img, (20, 20))
                pygame.draw.line(screen, WHITE, right_starting_point, left_starting_point)

                sub = grab(0, 40, 950, 950)
                surf = pygame.transform.rotate(sub, 90)
                surf = pygame.transform.flip(surf, False, True)
                screen.blit(surf, (0, 40))
        pygame.display.flip()
