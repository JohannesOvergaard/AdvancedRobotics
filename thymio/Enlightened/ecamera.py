# this imports the camera

#from picamera import PiCamera
import cv2
import numpy as np
import sys
from picamera import PiCamera

#initialize
IMG_WIDTH = 320
IMG_HEIGHT = 240
boundaries = [
    ([128, 149, 131], [179, 255, 255]), # red
    ([33, 28, 111], [80, 233, 255]), # green
    ([111, 90, 109], [138, 255, 217]), # blue 
    ([148, 50, 134], [164, 189, 229]), # purple 
    ([17, 145, 45], [31, 188, 235]) # yellow/orange )
]

# Init camera
print("Initializing camera...")
camera = PiCamera()
camera.start_preview()
camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
camera.framerate = 24
image = np.empty((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)

def capture():
    camera.capture(image, 'bgr')
    return cv2.rotate(image, cv2.ROTATE_180)

def stop_camera():
    camera.stop_preview()
    camera.close()


def get_direction_from_image(image_idx):
    # Partition image into [LEFT - CENTER - RIGHT]
    left_pixel_count = 0    # WIDTH: 0 - 106    HEIGHT: 0 - 79
    center_pixel_count = 0  # WIDTH: 107 - 213  HEIGHT: 80 - 159
    right_pixel_count = 0   # WIDTH: 214 - 319  HEIGHT: 160 - 239
    image = cv2.imread(f'out{image_idx+1}.png', cv2.IMREAD_GRAYSCALE)

    for column in image: # 240
        for idx, pixel in enumerate(column): # 320
            if (0 <= idx <= 106):
                left_pixel_count += pixel
            elif (107 <= idx <= 213):
                center_pixel_count += pixel
            elif (214 <= idx <= 319):
                right_pixel_count += pixel
    
    lst = np.array([left_pixel_count, center_pixel_count, right_pixel_count])
    DIRECTION =  np.argmax(lst)

    if (DIRECTION == 0):
        return "LEFT"
    elif (DIRECTION == 1):
        return "CENTER"
    elif (DIRECTION == 2):
        return "RIGHT"
    else:
        return "NONE"

def detect_color_and_direction(image):
    count = 0
    readings = []

    # Apply blurs to remove unnecessary details from image.
    blur = cv2.blur(image, (5,5))
    blur0 = cv2.medianBlur(blur, 5)
    cv2.imwrite('out_blur.png', blur0)

    #We prefer hsv color space, gives more accurate reading of each pixel - easier to define thresholds.
    hsv_image = cv2.cvtColor(blur0, cv2.COLOR_BGR2HSV) 
    cv2.imwrite('out_blur_hsv.png', hsv_image)
    
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        count += 1
        mask = None
        mask = cv2.inRange(hsv_image, lower, upper)
        cv2.imwrite('out_blur_mask.png', hsv_image)
        res = cv2.bitwise_and(hsv_image, hsv_image, mask= mask)

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
    DIRECTION = get_direction_from_image(i)

    if i == 0:
        return ("RED",  DIRECTION)
    if i == 1:
        return ("GREEN",  DIRECTION)
    if i == 2:
        return ("BLUE",  DIRECTION)
    if i == 3:
        return ("PURPLE",  DIRECTION)
    if i == 4:
        return ("YELLOW",  DIRECTION)

