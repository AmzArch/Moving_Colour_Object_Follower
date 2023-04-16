import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy
import os
from time import sleep
import numpy as np
from blob_interfaces.msg import BlobsList, Blob

class MovingBlobDetector(Node):

  def __init__(self):

    super().__init__('moving_blob_detector')
    #creating a ROS2 subscriber for the image topic which after listening performs the lister_callback callback function
    self.subscription = self.create_subscription(Image,'/image', self.moving_blob_callback, 1)
    #creating a publisher for the new topic detected_blobs
    self.blobs_pub = self.create_publisher(BlobsList, '/detected_blobs', 1)
    self.br = CvBridge()
    self.prev_frame = None


  def moving_blob_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)

    # Find moving areas in the current frame
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    if self.prev_frame is None:
        self.prev_frame = gray
    frame_diff = cv2.absdiff(self.prev_frame, gray)
    thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]

    # Update the previous frame for the next iteration
    self.prev_frame = gray


    # cv2.imshow("camera", current_frame)

    # Load image
    img = current_frame
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define range of green color in HSV
    lower_green = np.array([36, 25, 25])
    upper_green = np.array([86, 255, 255])

    # Define range of red color in HSV
    lower_red1 = np.array([0, 25, 25])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 25, 25])
    upper_red2 = np.array([180, 255, 255])

    # Define range of blue color in HSV
    lower_blue = np.array([100, 25, 25])
    upper_blue = np.array([130, 255, 255])

    # Threshold the HSV image to get only green colors
    mask_green = cv2.inRange(hsv_img, lower_green, upper_green)

    # Threshold the HSV image to get only red colors
    mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)

    # Apply moving area mask to all the color masks
    mask_green = cv2.bitwise_and(mask_green, thresh)
    mask_red = cv2.bitwise_and(mask_red, thresh)
    mask_blue = cv2.bitwise_and(mask_blue, thresh)

    # Find contours in the binary images
    contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw circles on the image for each detected blob
    green_blobs = []
    for contour in contours_green:
        area = cv2.contourArea(contour)
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 10, (0, 255, 0), -1)
            green_blobs.append((area, cx, cy))


    # Show isolated green blobs
    # cv2.imshow('Isolated Green Blobs', cv2.bitwise_and(img, img, mask=mask_green))

    red_blobs = []
    # Draw circles on the image for each detected blob
    for contour in contours_red:
        area = cv2.contourArea(contour)
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 10, (0, 0, 255), -1)
            red_blobs.append((area, cx, cy))

    # Show isolated red blobs
    # cv2.imshow('Isolated Red Blobs', cv2.bitwise_and(img, img, mask=mask_red))

    blue_blobs = []
    # Draw circles on the image for each detected blob
    for contour in contours_blue:
      area = cv2.contourArea(contour)
      M = cv2.moments(contour)
      if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 10, (255, 0, 0), -1)
            blue_blobs.append((area, cx, cy))

    # Show isolated red blobs
    # cv2.imshow('Isolated blue Blobs', cv2.bitwise_and(img, img, mask=mask_blue))
    
    # Sort all the color blob lists in orser of area
    red_blobs.sort()
    green_blobs.sort()
    blue_blobs.sort()

    #Using the BlosList Msg we create in blob_interfaces
    blobs_msg = BlobsList()

    for blob in red_blobs:
        new_blob = Blob()
        new_blob.color = 'red'
        new_blob.x = blob[1]
        new_blob.y = blob[2]
        new_blob.area = blob[0]
        blobs_msg.blobs.append(new_blob)

    for blob in green_blobs:
        new_blob = Blob()
        new_blob.color = 'green'
        new_blob.x = blob[1]
        new_blob.y = blob[2]
        new_blob.area = blob[0]
        blobs_msg.blobs.append(new_blob)

    for blob in blue_blobs:
        new_blob = Blob()
        new_blob.color = 'blue'
        new_blob.x = blob[1]
        new_blob.y = blob[2]
        new_blob.area = blob[0]
        blobs_msg.blobs.append(new_blob)

    self.blobs_pub.publish(blobs_msg)
    
    sleep(2)
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  moving_blob_detector = MovingBlobDetector()
  rclpy.spin(moving_blob_detector)
  moving_blob_detector.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

