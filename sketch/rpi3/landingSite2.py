# This code is based on semi-circle detection code written by Micka, a stackoverflow.com user
# Should anything happens, blame him :D 
import numpy as np
import cv2
import sys

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
 


class landingSite(object):
    # Image-wise parameters
    kernel_blur = np.ones((5,5),np.float32)/25
    kernel_morph = np.ones((2,2), np.uint8)
    image_width = 640
    image_height = 480
    min_area_ratio = 0.0009
    min_circle_radius = 20
    max_circle_radius = 300
    maxIterations = 25
    minCirclePercentage = 0.40
    
    # cv2.Canny parameters
    threshold1 = 100
    threshold2 = 50
    
    # Define range of blue/red color in HSV
    blue = np.array([[100,100,100], [130,230,230]]) # [lower threshold, upper threshold]
    red = np.array([[0, 100, 70], [25, 255, 255]])
    red_wrap = np.array([[163, 100, 70], [179, 255, 255]])
    
    def __init__(self, src):
#         print "NEW FRAME!"
        self.src = src.copy()
        self.original = src.copy()
        self.image_height, self.image_width, self.channels = np.shape(src) # Get frame dimension
        self.center = None
        self.radius = 0
        self.red = self.image_process(self.red, self.red_wrap)
        #self.blue = self.image_process(self.blue)
        
        # Show output
#         cv2.imshow('red', self.red)#         cv2.imshow('edges (red+blue)', self.red + self.blue) # Testing line, uncomment to test

        #cv2.imshow('src', self.src) # Testing line, uncomment to test
        
    def image_process(self, color, color_wrap=None):
        # Blur the image
#         blur = cv2.GaussianBlur(self.original, (3,3), 0)
                
        # Convert from BGR to HSV
        hsv = cv2.cvtColor(self.original.copy(), cv2.COLOR_BGR2HSV)
        
        if color_wrap is not None:
            lower_hue_red = cv2.inRange(hsv, color[0], color[1])
            upper_hue_red = cv2.inRange(hsv, color_wrap[0], color_wrap[1])
            dst = cv2.addWeighted(lower_hue_red, 1.0, upper_hue_red, 1.0, 0.0)
        else:
            dst = cv2.inRange(hsv, color[0], color[1])
        
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
            cv2.circle(self.src, (bestCircleCenter[0], bestCircleCenter[1]), bestCircleRadius, [0,255,0], 4) # Testing line, uncomment to test
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
        if radius < self.min_circle_radius or radius > self.max_circle_radius:
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
            if dt[int(cY), int(cX)] < maxInlierDist:
                inlier += 1
                inlierSet.append([cY, cX])
        # Return the percentage of circle found
        return float(inlier)/np.float(counter)
                    
if __name__ == '__main__':
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 20
        rawCapture = PiRGBArray(camera, size=(640, 480))
 
        # allow the camera to warmup
        time.sleep(0.1)
        print("cam init") 
        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
 
            # show the frame
            # cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF
            # Do the detection
            print landingSite(image).center
        
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
 
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
        cv2.destroyAllWindows()
