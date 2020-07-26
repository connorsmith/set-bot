#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('connor_set_bot')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
      cv2.CHAIN_APPROX_SIMPLE)[1]

    cv2.drawContours(cv_image, cnts, -1, (240, 0, 159), 3)

    # TODO shape detection
    # TODO identify colour / fill within contour
    # TODO associate contours to cards

    # show the annotated image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

class set_card:

  numbers = frozenset([1,2,3])
  colors = frozenset(['red', 'green', 'purple'])
  fills = frozenset(['solid', 'striped', 'empty'])
  shapes = frozenset(['oval', 'diamond', 'squiggle'])

  def __init__(self, number, color, fill, shape):
    self._number = number
    self._color = color
    self._fill = fill
    self._shape = shape

  def __eq__(self, set_card_object):
    return self._number == set_card_object._number and \
      self._color == set_card_object._color and \
      self._fill == set_card_object._fill and \
      self._shape == set_card_object._shape

class set_card_list:

  def __init__(self, cards = []):
    self._cards = cards

  def __contains__(self, search_card):
    for card in self.cards:
      if card == search_card:
        return True
    return False

  def sets(self):
    sets_found = []

    for card_index in len(self._cards):
      main_card = self._cards[card_index]
      to_be_checked = set_card_list(self._cards[card_index:])

      while to_be_checked:
        second_card = to_be_checked.pop()
        third_card = get_third_card(main_card, second_card)

        if third_card in to_be_checked:
          sets_found.append((main_card, second_card, third_card))

    return sets_found

def get_third_card(first_card, second_card):
  # figure out the number
  if first_card._number == second_card._number:
    third_card_number = first_card._number
  else:
    third_card_number = next(iter(set_card.numbers.difference(set)))

  # figure out the color
  if first_card._color == second_card._color:
    third_card_color = first_card._color
  else:
    third_card_color = next(iter(set_card.colors.difference(set)))

  # figure out the fill
  if first_card._fill == second_card._fill:
    third_card_fill = first_card._fill
  else:
    third_card_fill = next(iter(set_card.fills.difference(set)))

  # figure out the shape
  if first_card._shape == second_card._shape:
    third_card_shape = first_card._shape
  else:
    third_card_shape = next(iter(set_card.shapes.difference(set)))

  return set_card(third_card_number, third_card_color, third_card_fill, third_card_shape)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)