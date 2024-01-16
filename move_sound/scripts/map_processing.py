#!/usr/bin/env python3

from PIL import Image
import cv2
  
  
# sample.png is the name of the image
# file and assuming that it is uploaded
# in the current directory or we need
# to give the path
image = Image.open('/home/hello-robot/gaurav_ws/src/move_sound/maps/map2.yaml.pgm')
  
# summarize some details about the image
# print(image.format)
print(image.size)

# print(image.mode)
