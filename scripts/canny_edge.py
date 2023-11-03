import cv2
img = cv2.imread('/home/mpl/datasets/for visualization latex/indoor2/rgbfinal_1504645308.352415.png')
edges = cv2.Canny(img, 50, 150)
cv2.imwrite("/home/mpl/datasets/for visualization latex/indoor2_edge.png", edges)
