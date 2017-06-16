#!/usr/bin/python
import rospy
import roslib
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np
import time

G_LABEL_RIGHT = 0
G_LABEL_STRAIGHT = 1
G_LABEL_LEFT = 2

def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh

def onLeftSide(l, p):
    x1, y1 = l[0]
    x2, y2 = l[1]
    x, y = p
    v1 = (x2 - x1, y2 - y1)
    v2 = (x2 - x, y2 - y)
    cross_product = v1[0]*v2[1] - v1[1]*v2[0]
    if cross_product >= 0:
        return False
    else:
        return True

def decideLabel(scores):
    l = scores[0]
    m = scores[1]
    r = scores[2]

    if m <= 1:
        return G_LABEL_STRAIGHT
    if min(l, m, r) == m:
        return G_LABEL_STRAIGHT
    elif min(l, r) == l:
        return G_LABEL_LEFT
    else:
        return G_LABEL_RIGHT

class PedestrianDetector:

    def __init__(self):
        self.image_pub = rospy.Publisher("result_image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_raw", Image, self.img_callback)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def img_callback(self, data):
        start_time = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(cv_image, (50, 60), 10, 255)
        cv_image = imutils.resize(cv_image, width=min(400, cols))
        found, w = self.hog.detectMultiScale(cv_image, winStride=(8, 8), padding=(32, 32), scale=1.05)
        self.draw_detection(cv_image, found)
        label = self.decideAction(cv_image, found)
        if(label == 1):
            print("go straight")
        elif(label == 2):
            print("go left")
        else:
            print("go right")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        elapsed_time = time.time() - start_time
        print(elapsed_time * 1000)



    def decideAction(self, img, rects, thickness=1, color=(0, 255, 0)):
        # params for scoring
        SIZE_PENALTY = 10000 # for each 10000, add 1 penalty score

        # scores for each section
        scores = [0, 0, 0]

        # draw section divider
        color_d = (0, 0, 255)
        thick_d = 3
        iw = img.shape[1]
        ih = img.shape[0]

        # set ratio for central section on upper and lower edge
        upper_center_ratio = 1.0/5.0
        lower_center_ratio = 1.0/3.0

        # set section pivot points
        upper = int(iw/2.0 - upper_center_ratio*iw/2.0)
        lower = int(iw/2.0 - lower_center_ratio*iw/2.0)
        x11 = upper
        x12 = lower
        x21 = iw - upper
        x22 = iw - lower
        l1 = [(x11, 0), (x12, ih)]
        l2 = [(x21, 0), (x22, ih)]

        # cv2.line(img, l1[0], l1[1], color_d, thick_d)
        # cv2.line(img, l2[0], l2[1], color_d, thick_d)

        for x, y, w, h in rects:
            pad_w, pad_h = int(0.15*w), int(0.05*h)
            color_no_pad = (255, 255, 255)
            thickness_no_pad = 3

            # rect out the human
            # cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), color, thickness)

            # calculate area
            w_draw = w - 2*pad_w
            h_draw = h - 2*pad_h
            area = w_draw * h_draw

            # center of the human
            center = (x+int(0.5*w), y+int(0.5*h))
            # print(center)

            # decide which section the point belongs to
            left1 = onLeftSide(l1, center)
            left2 = onLeftSide(l2, center)
            if left1:
                idx = 0
            elif left2:
                idx = 1
            else:
                idx = 2

            # score for appearance
            scores[idx] += 1
            # score for area size
            scores[idx] += area/SIZE_PENALTY

            # cv2.circle(img, center, 1, color_no_pad, thickness_no_pad)

        # warning_h = int(ih/2)
        # warning_points = [(10, warning_h), (int(iw/2), warning_h), (iw-60, warning_h)]
        # for i in [0, 1, 2]:
        #     cv2.putText(img, "s: %d" % scores[i], warning_points[i], cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_d, 2)

        label = decideLabel(scores)
        # if label == G_LABEL_RIGHT:
        #     text = "Right"
        # elif label == G_LABEL_STRAIGHT:
        #     text = "Straight"
        # else:
        #     text = "Left"
        # cv2.putText(img, text, (int(iw/2-5), int(ih/2+100)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_d, 2)
        return label
    def draw_detection(self, img, rects, thickness=1, color=(0, 255, 0)):
        # print('')

        # draw section divider
        color_d = (0, 0, 255)
        thick_d = 3
        iw = img.shape[1]
        ih = img.shape[0]

        # set ratio for central section on upper and lower edge
        upper_center_ratio = 1.0/5.0
        lower_center_ratio = 1.0/3.0

        # print(upper_center_ratio)

        upper = int(iw/2.0 - upper_center_ratio*iw/2.0)
        lower = int(iw/2.0 - lower_center_ratio*iw/2.0)
        x11 = upper
        x12 = lower
        x21 = iw - upper
        x22 = iw - lower

        cv2.line(img, (x11, 0), (x12, ih), color_d, thick_d)
        cv2.line(img, (x21, 0), (x22, ih), color_d, thick_d)

        for x, y, w, h in rects:
            pad_w, pad_h = int(0.15*w), int(0.05*h)
            color_no_pad = (255, 255, 255)
            thickness_no_pad = 3

            # rect out the human
            cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), color, thickness)

            # center of the human
            center = (x+int(0.5*w), y+int(0.5*h))
            # print(center)
            cv2.circle(img, center, 1, color_no_pad, thickness_no_pad)



def main(args):
    rospy.init_node('PedestrianDetector', anonymous=True)
    ic = PedestrianDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
