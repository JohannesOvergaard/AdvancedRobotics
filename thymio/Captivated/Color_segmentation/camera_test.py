# this imports the camera

#from picamera import PiCamera
import cv2
import numpy as np
import sys
#initialize

#camera = PiCamera()

'''def testCamera():
    print("Camera test")
    camera.start_preview()
    sleep(5)
    #we capture to openCV compatible format
    #you might want to increase resolution
    camera.resolution = (240, 240)
    camera.framerate = 24
    sleep(2)
    image = np.empty((240, 240, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    cv2.imwrite('out.png', image)
    camera.stop_preview()   
    print("saved image to out.png")
'''

#boundaries defined in HSV format.
boundaries = [
    ([340, 70, 50], [359, 100, 100]), # red
    ([90, 70, 40], [130, 100, 100]), # green
    ([220, 50, 50], [250, 100, 100]), # blue 
    ([290, 50, 50], [310, 100, 100]), # purple 
    ([45, 50, 40], [60, 100, 100]) # yellow/orange 
]

readings = []

def detect_color(image):
    '''camera.resolution(240,240)
    camera.framerare = 24
    camera.start_preview()
    sleep(2)
    image = np.empty((240, 240, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')'''
    image = cv2.imread(image)
    
    count = 0
    # Apply blurs to remove unnecessary details from image.
    blur = cv2.blur(image, (5,5))
    cv2.imwrite('out_blur.png', blur)
    #blur0 = cv2.medianBlur(blur, 5)

    #We prefer hsv color space, gives more accurate reading of each pixel - easier to define thresholds.
    hsv_image = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) 

    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        count += 1
        mask = None
        mask = cv2.inRange(hsv_image, lower, upper)
        res = cv2.bitwise_and(hsv_image,hsv_image, mask= mask)

        #takes the average color of the image - if it is greater than 0.0 it will have detected some color in the image.
        avg_color_per_row = np.average(res, axis=0)
        avg_color = np.average(avg_color_per_row, axis=0)
        readings.append(avg_color)
        cv2.imwrite('out'+str(count)+'.png', res)
        
    averages = []
    #simplistic approach: the category for which the average of the rgb value is greatest is the one we have detected.
    for i in readings:
        sum = 0
        for q in i:
            sum += q
        avg = sum / 3
        averages.append(avg)

    i = averages.index(max(averages))
    if i == 0:
        print("red")
    if i == 1:
        print("green")
    if i == 2:
        print("blue")
    if i == 3:
        print("yellow")

a = sys.argv[1]
print(a)
detect_color(str(a))
