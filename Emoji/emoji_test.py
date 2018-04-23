#! /usr/bin/env python
# FYI: https://realpython.com/blog/python/face-recognition-with-python/
import cv2
import rospy

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

webcam = cv2.VideoCapture(1)


class Emoji:
    def __init__(self, fn):

        self.filename = fn
        emo = cv2.imread(self.filename, -1)
        # create the mask for each emoji object
        self.orig_mask = emo[:, :, 3]
        # creating an inverted mask for each emoji object
        self.orig_mask_inv =  cv2.bitwise_not(self.orig_mask)
        # Convert emo image to BGR
        # and save the original image size (used later when re-sizing the image)
        emo = emo[:, :, 0:3]
        self.origHeight, origWidth = emo.shape[:2]
        print "emoji created"
        print(self.filename)


# def display():
#     while True:
#         # capture frame by frame
#         ret, img = webcam.read()
#         if not img is None:
#             gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#             #reading the image and converting it into grayscale
#             faces = face_cascade.detectMultiScale(
#                 gray,
#                 scaleFactor=1.3,
#                 minNeighbors=5
#             )
#             for (x, y, w, h) in faces:
#                 # Draw a rectangle around each face
#                 #cv2.rectangle(img, (x,y), (x+w, y+h), (135, 135, 0), 2)
#                 #roi_gray = gray[y:y+h, x:x+w]
#                 #roi_color = img[y:y+h, x:x+w]
#
#                 #calculate size of emoji
#                 emoW = h
#                 emoH = h
#                 y1 = y
#                 y2 = y+h
#                 x1 = x
#                 x2 = x+w
#                 #resizing the original image and the masks to the new emoji sizes
#                 emoji = cv2.resize(emo_list[0],(emoW,emoH),interpolation=cv2.INTER_AREA)
#                 mask = cv2.resize(orig_mask,(emoW,emoH),interpolation=cv2.INTER_AREA)
#                 mask_inv=cv2.resize(orig_mask_inv,(emoW,emoH),interpolation=cv2.INTER_AREA)
#
#                 # take ROI for emoji from background equal to size of emoji image
#                 roi = img[y1:y2, x1:x2]
#
#                 # roi_bg contains the original image only where the emoji is not
#                 # in the region that is the size of the mustache.
#                 roi_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
#
#                 # roi_fg contains the image of the mustache only where the mustache is
#                 roi_fg = cv2.bitwise_and(emoji, emoji, mask=mask)
#
#                 # join the roi_bg and roi_fg
#                 dst = cv2.add(roi_bg, roi_fg)
#
#                 # place the joined image, saved to dst back over the original image
#                 img[y1:y2, x1:x2] = dst
#                 break
#
#
#         #Displaying the resulting frame
#         cv2.imshow('img', img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#
#     #When everything is done, release the capture
#     webcam.release()
#     # output_video.release()
#     cv2.destroyAllWindows()



def main():
    emo_list = []
    for i in range(0, 5):
        # testing emoji overlay
        pic_list = []
        if i == 0:
            file_emo = "angry"
        elif i == 1:
            file_emo = "disgust"
        elif i == 2:
            file_emo = "happy"
        elif i == 3:
            file_emo = "meh"
        elif i == 4:
            file_emo = "tt"
        for j in range(1, 5):
            file_name = "../images/" + file_emo + str(j) + ".png"
            pic_list.append(Emoji(file_name))
        emo_list.append(pic_list)
    display()

if __name__ == '__main__':
    main()
