
import numpy as np
import cv2

config={}
config["img_path"]=""
config["K"]=np.zeros([3,3])
config["dist"]=np.zeros(5)

checkerboard_row = 6
checkerboard_col = 9
checkerboard_size = 0.0335  # meter
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

imgpoints = []  # 2d points in image plane.

img = cv2.imread(config["img_path"])
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (6, 9), None)
if ret == True:
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners)
    # Draw and display the corners
    # cv2.drawChessboardCorners(img, (6,9), corners2, ret)
    # cv2.imwrite('img.jpg', img)


cam = []
cameraMatrix = config["K"]
distCoeffs = config["dist"]

for i in imgpoints[0]:
    cam.append(i[0].reshape(1, 2)[0])

cam = np.array(cam)

objectPoints = np.zeros((checkerboard_row*checkerboard_col, 3))

for i in range(0, checkerboard_col):
    for j in range(checkerboard_row):
        objectPoints[i*checkerboard_row+j][0] = i*checkerboard_size
        objectPoints[i*checkerboard_row+j][1] = (5-j)*checkerboard_size
        objectPoints[i*checkerboard_row+j][2] = 0

retval, rvec, tvec = cv2.solvePnP(objectPoints, cam, cameraMatrix, distCoeffs)
print(cv2.Rodrigues(rvec))
print(tvec)
