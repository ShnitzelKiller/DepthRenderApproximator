import sys
import cv2
import os.path
from color_transfer import color_transfer

if len(sys.argv) < 3:
    print("usage: " + sys.argv[0] + " filename1 filename2")
    exit()

if len(sys.argv) > 3:
    filename = sys.argv[3]
else:
    head, tail = os.path.split(sys.argv[1])
    name, ext = os.path.splitext(tail)
    filename = tail + "_transformed" + ext

img1 = cv2.imread(sys.argv[1])
img2 = cv2.imread(sys.argv[2])

img3 = color_transfer(img2, img1)

cv2.imwrite(filename, img3)