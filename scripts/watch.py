#!/usr/bin/python

import sys,pygame,math,time

import rospy
from lightsensor_gazebo.msg import LightSensor,Pixel

    
rospy.init_node('watch',anonymous=True)

pygame.init()
size=width,height=320,320
black=0,0,0
screen = pygame.display.set_mode(size)
radius = width*0.4
font = pygame.font.Font(None,18)

def convcol(v):
    v=v+20
    if v>255: v=255
    return v

def callback(data):
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
    screen.fill(black)
    
    ct = len(data.pixels)
    step = (2*math.pi)/ct
    angle = 0    
    n=0
    sqsize = math.tan(step)*radius
    tickstep = ct/10
    for p in data.pixels:
        x = -math.sin(angle)*radius
        y = math.cos(angle)*radius
        pygame.draw.rect(screen,
            [convcol(p.r),convcol(p.g),convcol(p.b)],
            [x+width/2-sqsize/2,y+height/2-sqsize/2,sqsize,sqsize])
        if n%tickstep == 0:
            x = -math.sin(angle)*(radius+8)
            y = math.cos(angle)*(radius+8)
            text = font.render(str(n),1,(255,255,255))
            textpos = text.get_rect()
            textpos.centerx=x+width/2
            textpos.centery=y+height/2
            screen.blit(text,textpos)
            
        
        angle = angle+step
        n=n+1
    pygame.display.flip()



rospy.Subscriber("light",LightSensor,callback)

rospy.spin()

