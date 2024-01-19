import cv2 as cv
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt


def getImgPaths():
    images = []
    for filename in os.listdir('./imgs'):
        images.append('./imgs/'+filename);
    return images;

def contour_area(contours):
     
    # create an empty list
    cnt_area = []
    # loop through all the contours
    for i in range(0,len(contours),1):
        # for each contour, use OpenCV to calculate the area of the contour
        cnt_area.append(cv.contourArea(contours[i]))
 
    # Sort our list of contour areas in descending order
    list.sort(cnt_area, reverse=True)
    return cnt_area



#main method
if __name__ == '__main__':
    i = 0;
    for path in getImgPaths():
        i+=1;
        img = cv.imread(path); #this should come in bgr8 format
        #img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        vest_lower = np.array([28, 36, 55])
        vest_upper = np.array([50, 255, 243]) 

        #(hMin = 28 , sMin = 36, vMin = 55), 
        #(hMax = 50 , sMax = 255, vMax = 243)

        mask = cv.inRange(hsv_img, vest_lower, vest_upper);
        masked = cv.bitwise_and(hsv_img, hsv_img, mask=mask);

        masked = cv.cvtColor(masked, cv.COLOR_HSV2RGB);


        fig = plt.figure(figsize=(10, 10));
        
        ax = fig.add_subplot(1, 2, 1);
        ax.set_title('original picture');
        ax.imshow(img[:,:,::-1], cmap="gray");
        ax.set_xticks([]); ax.set_yticks([]);
       
        ax = fig.add_subplot(1, 2, 2);
        ax.set_title('masked image');
        ax.imshow(masked[:,:,::-1], cmap="gray");
        ax.set_xticks([]); ax.set_yticks([]);

        masked = cv.cvtColor(masked, cv.COLOR_RGB2GRAY);
        # plt.show();
        

        contours,hierarchy = cv.findContours(masked,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

        #create a numpy array of contourArea, and then use argmax
        #pull out contour of largest area
        #call cv.boundingRect on largest contour

        cnt_area = [];
        for i in range(len(contours)):
            cnt_area.append(cv.contourArea(contours[i]));
        max_ind = cnt_area.index(max(cnt_area));

        #print(cnt_area);
        #print("max index is", max_ind, "area is", cnt_area[max_ind]);

        x,y,w,h = cv.boundingRect(contours[max_ind]);
        
        #print("stuff", x, y, w, h)
        print("IMAGE", i)
        print("centroid of bounding box is", x+w//2, y+h//2);
        print("bounding box width", w);
        print("bounding box width", h);
        #TODO:
        #So, I'm guessing we can just do some basic proportional control with this
        #have the stimulus be based on distane btwn centroid and cenral vertical axis
        #of the camera feed image
      
        rect = matplotlib.patches.Rectangle((x, y), w, h,
                                     fill=False, edgecolor='red', linewidth=2)
        plt.gca().add_patch(rect)

        plt.imshow(img[:,:,::-1], cmap="gray");
        plt.scatter([x+w//2], [y+h//2], c='r');
        plt.show()
