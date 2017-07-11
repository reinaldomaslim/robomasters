# This code is based on semi-circle detection code written by Micka, a stackoverflow.com user
# Should anything happens, blame him :D 
import numpy as np
import cv2
import sys

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
        self.red = self.image_process(self.red, self.red_wrap)
        self.blue = self.image_process(self.blue)
        
        # Show output
#         cv2.imshow('red', self.red)
        cv2.imshow('edges (red+blue)', self.red + self.blue) # Testing line, uncomment to test
        cv2.imshow('src', self.src) # Testing line, uncomment to test
        
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
                # The next 12 lines are used to find all the circles present
# #                 print "accepted circle with ", cPerc*100.0, " % inlier" # Testing line, uncomment to test
#                 # first step would be to approximate the circle iteratively from ALL INLIER to obtain a better circle center
#                 # but that's a TODO
# #                 print "circle: center: ", center, " radius: ", radius # Testing line, uncomment to test
#                 cv2.circle(self.src, (center[0], center[1]), radius, [0,255,0], 4) # Testing line, uncomment to test
#                   
#                 # Remove accepted circle from list
#                 cv2.circle(edges, (center[0], center[1]), radius, [0], 10);
#    
#                 # Update edge positions and distance transform
#                 edgePositions = np.nonzero(edges)
#                 dt = cv2.distanceTransform(255-edges, cv2.cv.CV_DIST_L1, 3)
                #---------------------------------
                # The next 4 lines are used to find the best circle, only choose one option and comment out the other option
                # Also some comments out of this if also
                if cPerc > bestCirclePercentage:
                    bestCirclePercentage = cPerc
                    bestCircleCenter = center
                    bestCircleRadius = radius    
                
#             tmp = edges.copy()
#             cv2.imshow('src', self.src) # Testing line, uncomment to test
#             cv2.imshow('edges', edges) # Testing line, uncomment to test
#             cv2.waitKey(0) # Testing line, uncomment to test
        if bestCirclePercentage > 0.1:
#             print bestCirclePercentage # Testing line, uncomment to test
#             print "circle: center = ", bestCircleCenter, " radius = ", bestCircleRadius # Testing line, uncomment to test
            cv2.circle(self.src, (bestCircleCenter[0], bestCircleCenter[1]), bestCircleRadius, [0,255,0], 4) # Testing line, uncomment to test
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
        
#         print [x1,y1], [x2,y2], [x3,y3], " center: ", center, " radius ", radius # Testing line, uncomment to test
#         tmp = self.src.copy() # Testing line, uncomment to test
#         cv2.circle(tmp, (center[0], center[1]), np.int32(radius), [0,100,100], 4) # Testing line, uncomment to test
#         cv2.circle(tmp, (y1, x1), 4, [255,0,255], -1) # Testing line, uncomment to test
#         cv2.circle(tmp, (y2, x2), 4, [255,0,255], -1) # Testing line, uncomment to test
#         cv2.circle(tmp, (y3, x3), 4, [255,0,255], -1) # Testing line, uncomment to test
#         cv2.waitKey(-1) # Testing line, uncomment to test
#         cv2.imshow('src', tmp) # Testing line, uncomment to test
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
    # Instructions
    print "Press q to exit, p to pause."
    
    # Windows for output
#     cv2.namedWindow('red', cv2.WINDOW_NORMAL)
#     cv2.namedWindow('blue', cv2.WINDOW_NORMAL)
#     cv2.namedWindow('src', cv2.WINDOW_NORMAL)
#     cv2.moveWindow('src', 0, 0)
     
    # Take in file
    if len(sys.argv) < 2:
        print "Please enter an video input."
        sys.exit()
        
    # Read file
    cap = cv2.VideoCapture(sys.argv[1])
    
    frame_counter = 0 # Used to loop video
    while(cap.isOpened()):
        # Get frame from video file
        retval, src = cap.read()
        frame_counter += 1
        
        # If the last frame is reached, reset the capture and the frame_counter
        if frame_counter == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT):
            frame_counter = 0
            cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)
            
        # Do the detection
        landingSite(src)

        # User input
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()