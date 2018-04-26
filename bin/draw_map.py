#!/usr/bin/env python

import cv2
import numpy as np

drawing = False # true if mouse is pressed
erasing = False # true if rmouse is pressed
mode = False # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
brush_size = 15

def nothing(x):
    pass

# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode, erasing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.circle(img,(x,y), brush_size, 255,-1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.circle(img,(x,y), brush_size, 255,-1)
            
    if event == cv2.EVENT_RBUTTONDOWN:
        erasing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if erasing == True:
            cv2.circle(img,(x,y), brush_size, 0,-1)

    elif event == cv2.EVENT_RBUTTONUP:
        erasing = False
        cv2.circle(img,(x,y), brush_size, 0,-1)


if __name__ == '__main__':
    
    img = np.zeros((512, 512, 1), np.uint8)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('image',draw_circle)
    
    cv2.createTrackbar('Brush Size','image',0,100,nothing)
    cv2.setTrackbarPos('Brush Size','image', brush_size)
    
    while(1):
        brush_size = cv2.getTrackbarPos('Brush Size','image')
        cv2.imshow('image',img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            cv2.imwrite('map.png', img)
            break
    
    cv2.destroyAllWindows()