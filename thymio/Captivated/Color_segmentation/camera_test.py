# this imports the camera

#from picamera import PiCamera
import cv2
import numpy as np

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
    ([0, 100, 50], [10, 255, 200]), # red
    ([30, 100, 50], [90, 255, 200]), # green
    ([70, 100, 50], [100, 255, 200]), # blue
    ([20, 100, 50], [30, 255, 200]) # yellow
]

readings = []

def detect_color():
    '''camera.resolution(240,240)
    camera.framerare = 24
    camera.start_preview()
    sleep(2)
    image = np.empty((240, 240, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')'''
    image = cv2.imread('./yellow.jpg')
    
    count = 0
    # Apply blurs to remove unnecessary details from image.
    blur = cv2.blur(image, (5,5))
    blur0 = cv2.medianBlur(blur, 5)

    #We prefer hsv color space, gives more accurate reading of each pixel - easier to define thresholds.
    hsv_image = cv2.cvtColor(blur0, cv2.COLOR_BGR2HSV) 

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

detect_color()
