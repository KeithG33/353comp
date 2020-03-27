import cv2
import numpy as np

# import image
image = cv2.imread('C:/Users/zackw/Desktop/353/enph353_cnn_lab/plate_KM75.png')

colour_cut = cv2.inRange(image, 244,255)
cv2.imshow('colourcut', colour_cut)
thresh = colour_cut

kernel = np.ones((10, 1), np.uint8)
img_dilation = cv2.dilate(thresh, kernel, iterations=1)
cv2.imshow('dilated', img_dilation)


# find contours
# cv2.findCountours() function changed from OpenCV3 to OpenCV4: now it have only two parameters instead of 3
cv2MajorVersion = cv2.__version__.split(".")[0]
# check for contours on thresh
if int(cv2MajorVersion) >= 4:
    ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
else:
    im2, ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# sort contours
sorted_ctrs = sorted(ctrs, key=lambda ctr: cv2.boundingRect(ctr)[0])

for i, ctr in enumerate(sorted_ctrs):
    # Get bounding box
    x, y, w, h = cv2.boundingRect(ctr)

    # Getting ROI
    roi = image[y:y + h, x:x + w]

    # show ROI
    # cv2.imshow('segment no:'+str(i),roi)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
   
    if w > 15 and h > 15:
        cv2.imwrite('C:\\Users\\zackw\\Desktop\\353\\enph353_cnn_lab\\out\\{}.png'.format(i), roi)
        letter = image[x:x+w,y:y+h]
        cv2.imwrite('C:\\Users\\zackw\\Desktop\\353\\enph353_cnn_lab\\out\\letter\\{}.png'.format(i), roi)


cv2.imshow('marked areas', image)
cv2.waitKey(0)