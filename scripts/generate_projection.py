import cv2
import numpy as np

fs = cv2.FileStorage("../esvo_plus/cfg/system_mpl.yaml",
                     cv2.FILE_STORAGE_READ)

K_ev = fs.getNode("K_ev").mat()

dist_ev = fs.getNode("dist_ev").mat()

K_ev = np.array(K_ev).reshape([3, 3])

dist_ev = np.array(dist_ev)

print(K_ev)

print(dist_ev)

P = cv2.getOptimalNewCameraMatrix(K_ev, dist_ev, (640, 480), 0)

print(P[0])

print(P[1])

fs.release()
