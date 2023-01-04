import math
import random
import cvzone
import cv2
import numpy as np
import HandTrackingModule as htm
import time
import autopy

wcam , hcam = 1000,780
frameR = 100 # Frame Reduction
smoothening = 6

pTime = 0
plocX, plocY = 0, 0
clocX, clocY = 0, 0

cap = cv2.VideoCapture(0)
cap.set(3, wcam)
cap.set(4, hcam)
detector = htm.handDetector(maxHands=1)
wScr, hScr = autopy.screen.size()

class SnakeGameClass:
    def __init__(self, pathFood):
        self.points = []  # all points of the snake
        self.lengths = []  # distance between each point
        self.currentLength = 0  # total length of the snake
        self.allowedLength = 200  # total allowed Length
        self.previousHead = 0, 0  # previous head point

        self.imgFood = cv2.imread(pathFood, cv2.IMREAD_UNCHANGED)
        self.hFood, self.wFood, _ = self.imgFood.shape
        self.foodPoint = 0, 0
        self.randomFoodLocation()

        self.score = 0
        self.gameOver = False

    def randomFoodLocation(self):
        self.foodPoint = random.randint(100, 1000), random.randint(100, 600)

    def update(self, imgMain, currentHead):

        if self.gameOver:
            cv2.putText(imgMain, "Game Over", [300, 400],cv2.FONT_HERSHEY_SIMPLEX,
                               fontscale=7, thickness=5, offset=20)
            cv2.putText(imgMain, f'Your Score: {self.score}', [300, 550],cv2.FONT_HERSHEY_SIMPLEX,
                               fontscale=7, thickness=5, offset=20)

        else:
            px, py = self.previousHead
            cx, cy = currentHead
    
            self.points.append([cx, cy])
            distance = math.hypot(cx - px, cy - py)
            self.lengths.append(distance)
            self.currentLength += distance
            self.previousHead = cx, cy
    
            # Length Reduction
            if self.currentLength > self.allowedLength:
                for i, length in enumerate(self.lengths):
                    self.currentLength -= length
                    self.lengths.pop(i)
                    self.points.pop(i)
                    if self.currentLength < self.allowedLength:
                        break
    
            # Check if snake ate the Food
            rx, ry = self.foodPoint
            if rx - self.wFood // 2 < cx < rx + self.wFood // 2 and \
                    ry - self.hFood // 2 < cy < ry + self.hFood // 2:
                self.randomFoodLocation()
                self.allowedLength += 50
                self.score += 1
                print(self.score)
        
            # Draw Snake
            if self.points:
                for i, point in enumerate(self.points):
                    if i != 0:
                        cv2.line(imgMain, self.points[i - 1], self.points[i], (0, 0, 255), 20)
                cv2.circle(imgMain, self.points[-1], 20, (0, 255, 0), cv2.FILLED)
            
            
    
    
            # Draw Food
            rx, ry = self.foodPoint
            imgMain = cvzone.overlayPNG(imgMain, self.imgFood,(rx - self.wFood // 2, ry - self.hFood // 2))
    
            cv2.putText(imgMain, f'Score: {self.score}', [50, 80],cv2.FONT_HERSHEY_SIMPLEX,fontscale=3, thickness=3, offset=10)
    
    
            # Check for Collision
            pts = np.array(self.points[:-2], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(imgMain, [pts], False, (0, 255, 0), 3)
            minDist = cv2.pointPolygonTest(pts, (cx, cy), True)
            if -1 <= minDist <= 1:
                print("Hit")
                self.gameOver = True
                self.points = []  # all points of the snake
                self.lengths = []  # distance between each point
                self.currentLength = 0  # total length of the snake
                self.allowedLength = 150  # total allowed Length
                self.previousHead = 0, 0  # previous head point
                self.randomFoodLocation()
    
            
            return imgMain

game = SnakeGameClass("Donut.png")


while True:
    success,img = cap.read()
    img = cv2.flip(img, 1)
    img = detector.findHands(img)
    lmList,bbox= detector.findPosition(img)

    if len(lmList) != 0:
        x1, y1 = lmList[8][1:]
        x2, y2 = lmList[12][1:]
        #pointIndex = lmList[8][0:2]
        # print(x1, y1, x2, y2)
        img = game.update(img,(x1,y1))

        cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)




    cv2.imshow("Image", img)
    key = cv2.waitKey(1)
    if key == ord('r'):
        game.gameOver = False
