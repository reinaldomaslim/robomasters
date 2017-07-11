# UAV LED landing assistant system on RPi3
# NTU Mecatron 
# Tuan Anh, Jerriel, Yan Pai, Reinaldo

import numpy as np
import cv2
import sys
import time
import math 
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from neopixel import *

# LED strip configuration:
LED_COUNT      = 16      # Number of LED pixels.
LED_PIN1        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_PIN2        = 19
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL0    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_CHANNEL1    = 1
LED_STRIP      = ws.WS2811_STRIP_GRB   # Strip type and colour ordering


# Define functions which animate LEDs in various ways.
# if within 40 radius light all green, safe to land
safe_landing=40


def LED_Signal(strip11, strip12, strip21, strip22, wait_ms=100.0):
        
    """Wipe color across display a pixel at a time."""
    
    """First strip"""
    for i in range(0, LED_COUNT/2):
            strip1.setPixelColor(i, color[strip11])
    for i in range(LED_COUNT/2, LED_COUNT):
            strip1.setPixelColor(i, color[strip12])

    """Second strip"""
    for i in range(0, LED_COUNT/2):
            strip2.setPixelColor(i, color[strip21])
    for i in range(LED_COUNT/2, LED_COUNT):
            strip2.setPixelColor(i, color[strip22])


    strip1.show()
    strip2.show() 
    time.sleep(wait_ms/1000.0)

        
def toggleLED(center, radius):
    print radius
    if radius<15:
            #turn all red
            LED_Signal(0, 0, 0, 0)
    else:
            if center[0]<(image_width/2)-safe_landing:
                    left=1
                    right=0
            elif center[0]>(image_width/2)+safe_landing:
                    left=0
                    right=1
            else:
                    left=4
                    right=4
                    
            if center[1]<(image_height/2)-safe_landing:
                    down=1
                    top=0
            elif center[1]>(image_height/2)+safe_landing:
                    down=0
                    top=1
            else:
                    down=4
                    top=4                        
            
            LED_Signal(left, right, down, top)
            
class landingSite(object):
    # Image-wise parameters
    kernel_blur = np.ones((5,5),np.float32)/25
    kernel_morph = np.ones((2,2), np.uint8)
    image_width = 640/2
    image_height = 480/2
    min_area_ratio = 0.02
    min_circle_radius = 20
    max_circle_radius = 130
    maxIterations = 15
    minCirclePercentage = 0.2
    
    # cv2.Canny parameters
    threshold1 = 100
    threshold2 = 50
    
    def __init__(self):
        #print "NEW FRAME!"
        print("init_detection")

    def new_image(self, src):

        
        self.src = cv2.pyrDown(src.copy())
        self.original = self.src.copy()
        self.image_height, self.image_width, self.channels = np.shape(self.src) # Get frame dimension
        global image_height, image_width
        image_height=self.image_height
        image_width=self.image_width
        self.center = None
        self.radius = 0
        self.edge = self.image_process()
        
        # Show output

        #cv2.imshow('edges (red+blue)', self.edge) # Testing line, uncomment to test
        #cv2.imshow('src', self.src) # Testing line, uncomment to test

        
    def image_process(self):
        # Blur the image
        # blur = cv2.GaussianBlur(self.original, (3,3), 0)
                
        # Convert from BGR to HSV
        hsv = cv2.cvtColor(self.src, cv2.COLOR_BGR2HSV)
        lower_hue_red = cv2.inRange(hsv, (0, 100, 70), (25, 255, 255))
        upper_hue_red = cv2.inRange(hsv, (163, 100, 70), (179, 255, 255))
        red_binary = cv2.addWeighted(lower_hue_red, 1.0, upper_hue_red, 1.0, 0.0)
        blue_binary = cv2.inRange(hsv, (80,80,80), (130,255,255))
        dst = cv2.addWeighted(red_binary, 1.0, blue_binary, 1.0, 0.0)
        # Smooth the image

        dst = cv2.morphologyEx(dst, cv2.MORPH_OPEN, self.kernel_morph)
        dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, self.kernel_morph)
        
        contours, _ = cv2.findContours(dst.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process each blob found
        for idx, cnt in enumerate(contours):
            cnt_area = cv2.contourArea(cnt)
               
            # Delete blobs that are too small
            if cnt_area < (self.image_height*self.image_width)*self.min_area_ratio:
                cv2.drawContours(dst, contours, idx, np.array([0]), thickness=cv2.cv.CV_FILLED)
    
        edges = cv2.Canny(dst, self.threshold1, self.threshold2)
        # Find circles
        self.RANSAC(edges.copy())      
        return edges
    
    def RANSAC(self, edges):
        # edgePositions is a tuple of format (row, column)
        # This is the getPointPositions function of Micka's code
        edgePositions = np.nonzero(edges)
        
        if edgePositions is None:
            return
        
        # Create distance transform to efficiently evaluate distance to nearest edge
        dt = cv2.distanceTransform(255-edges, cv2.cv.CV_DIST_L1, 3)
        # TODO: maybe seed random variable for real random numbers.
        
        bestCirclePercentage = 0.0
        bestCircleCenter = None
        bestCircleRadius = None   
        for _ in xrange(self.maxIterations):
            # RANSAC: randomly choose 3 point and create a circle:
            # TODO: choose randomly but more intelligent 
            # so that it is more likely to choose three points of a circle.
            # For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
            if np.shape(edgePositions)[1] == 0:
                continue
            idx1 = np.random.randint(0, np.shape(edgePositions)[1])
            idx2 = np.random.randint(0, np.shape(edgePositions)[1])
            idx3 = np.random.randint(0, np.shape(edgePositions)[1])
            
            # We need 3 different samples
            if idx1 == idx2 or idx1 == idx3 or idx2 == idx3:
                continue
            
            # Create circle from 3 points
            center, radius = self.getCircle(edgePositions[0][idx1], edgePositions[1][idx1],
                                            edgePositions[0][idx2], edgePositions[1][idx2],
                                            edgePositions[0][idx3], edgePositions[1][idx3])
            
            if center is None or radius is None:
                continue
            
            # TODO: should be a set of points of the new circle
            # Nothing is here yet, in place for more development
            inlierSet = []
            
            # Verify or falsify the circle by inlier counting
            cPerc = self.verifyCircle(dt, center, radius, inlierSet)
#             print "cPerc = ", cPerc # Testing line, uncomment to test
            # If found any circles
            if cPerc >= self.minCirclePercentage:
                if cPerc > bestCirclePercentage:
                    bestCirclePercentage = cPerc
                    bestCircleCenter = center
                    bestCircleRadius = radius    
                
        if bestCirclePercentage > 0.1 and bestCircleRadius > self.radius:
#             print bestCirclePercentage # Testing line, uncomment to test
#             print "circle: center = ", bestCircleCenter, " radius = ", bestCircleRadius # Testing line, uncomment to test
            cv2.circle(self.src, (bestCircleCenter[0], bestCircleCenter[1]), bestCircleRadius, [0,255,0], 2) # Testing line, uncomment to test
            self.center = bestCircleCenter
            self.radius = bestCircleRadius

        return

    def getCircle(self, x1, y1, x2, y2, x3, y3):
        center = [0, 0] # [x, y] or [column, row]
        # Find circle from 3 points, Cartesian
        d = 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2)
        if d == 0:
            return None, None
        # Column, x
        center[0] = ((x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1))/d
        # Row, y
        center[1] = ((x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2))/d
        # Radius
        radius = np.sqrt((center[1]-x1)*(center[1]-x1) + (center[0]-y1)*(center[0]-y1))
        if(radius < self.min_circle_radius or radius > self.max_circle_radius
           or np.isnan(center[0]) or np.isnan(center[1])):
            return None, None
        
        return np.int32(center), np.float32(radius)

    def verifyCircle(self, dt, center, radius, inlierSet):
        counter = 0
        inlier = 0
        minInlierDist = 1.0
        maxInlierDistMax = 100.0
        maxInlierDist = radius/25.0
        if maxInlierDist < minInlierDist:
            maxInlierDist = minInlierDist
        if maxInlierDist > maxInlierDistMax:
            maxInlierDist = maxInlierDistMax
            
        # Choose samples along the circle and count inlier percentage
        angle = np.arange(0.0, 2*np.pi, 0.05)
        for t in angle:
            counter += 1
            cX = radius*np.cos(t) + center[0] # x-column
            cY = radius*np.sin(t) + center[1] # y-row
            if (cX < 0 or cX >= self.image_width
                or cY < 0 or cY >= self.image_height):
                continue

            if math.isnan(cX) or math.isnan(cY):
                continue
            
            if dt[int(cY), int(cX)] < maxInlierDist:
                inlier += 1
                inlierSet.append([cY, cX])
        # Return the percentage of circle found
        return float(inlier)/np.float(counter)
                    
if __name__ == '__main__':
    try:

            # initialise led        
            strip1 = Adafruit_NeoPixel(LED_COUNT, LED_PIN1, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL0, LED_STRIP)
            # Intialize the library (must be called once before other functions).
            strip1.begin()
            strip2 = Adafruit_NeoPixel(LED_COUNT, LED_PIN2, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL1, LED_STRIP)
            strip2.begin()

            Red = Color(255, 0 ,0)
            Green = Color(0, 255, 0)
            White = Color(255, 255, 255)
            No_color = Color(0, 0, 0)
            Blue = Color(0, 0, 255)
            color = [Red, Green, White, No_color, Blue]
            enumerate(color)
            
            # initialize the camera and grab a reference to the raw camera capture
            camera = PiCamera()
            camera.resolution = (640, 480)
            camera.framerate = 32
            rawCapture = PiRGBArray(camera, size=(640, 480))

            # allow the camera to warmup
            time.sleep(0.1)        
            print("cam init")
            LED_Signal(4, 4, 4, 4)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(4, 4, 4, 4)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(4, 4, 4, 4)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(4, 4, 4, 4)

            detect=landingSite()
            previous_center=None
            previous_radius=None
            # capture frames from the camera
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                    # grab the raw NumPy array representing the image, then initialize the timestamp
                    # and occupied/unoccupied text
                    image = frame.array

                    # show the frame
                    # cv2.imshow("Frame", image)
                    #key = cv2.waitKey(1) & 0xFF
                    # Do the detection
                    detect.new_image(image)
    
                    
                    if detect.center is None:
                        detected_center=previous_center
                        detected_radius=previous_radius
                    else:
                        detected_center=detect.center
                        detected_radius=detect.radius
                        
                    previous_center=detect.center
                    previous_radius=detect.radius
                    
                    print detect.center, detected_center
                    
                    toggleLED(detected_center, detected_radius)
                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            #if key == ord("q"):
                    #break
    except KeyboardInterrupt:
            cv2.destroyAllWindows()
            LED_Signal(2, 2, 2, 2)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(2, 2, 2, 2)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(2, 2, 2, 2)
            LED_Signal(3, 3, 3, 3)
            LED_Signal(2, 2, 2, 2)
            LED_Signal(3, 3, 3, 3)
            print()
            print('Successfully quit')
