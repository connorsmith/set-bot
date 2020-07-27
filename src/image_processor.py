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
from collections import OrderedDict
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

    colors = OrderedDict({
      "red": (255, 0, 0),
      "green": (0, 200, 0),
      "purple": (75, 0, 130)})

    # allocate memory for the L*a*b* image, then initialize
    # the color names list

    self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
    self.colorNames = []

    # loop over the colors dictionary
    for (i, (name, rgb)) in enumerate(colors.items()):
      # update the L*a*b* array and the color names list
      self.lab[i] = rgb
      self.colorNames.append(name)

    self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)

    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    lab_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

    thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
      cv2.CHAIN_APPROX_SIMPLE)[1]

    cv2.drawContours(cv_image, cnts, -1, (240, 0, 159), 3)

    cnt_dict = {}

    # loop over the contours
    for cnt_index, c in enumerate(cnts):
      # compute the center of the contour, then detect the name of the
      # shape using only the contour

      # TODO shape detection\

      local_lab_image = lab_image
      mask = np.zeros(local_lab_image.shape[:2], dtype="uint8")
      cv2.drawContours(mask, [c], -1, 255, -1)
      mask = cv2.erode(mask, None, iterations=2)
      mean = cv2.mean(local_lab_image, mask=mask)[:3]

      # initialize the minimum distance found thus far
      minDist = (np.inf, None)

      # cv2.imshow("Mask window", mask)
      # cv2.waitKey(250)

      # print("contour %s, mean %s"%(cnt_index, mean))

      # print(self.lab)
      contour_area = cv2.contourArea(c)
      if contour_area < 1000:
        # skip small shapes
        continue

      if contour_area < 1500:
        cnt_shape = 'diamond'
      elif contour_area < 2000:
        cnt_shape = 'squiggle'
      else:
        cnt_shape = 'oval'
      # contour_perimeter = cv2.arcLength(c, True)

      # print('Contour %s, area %s, perimeter %s'%(cnt_index, contour_area, contour_perimeter))

      # loop over the known L*a*b* color values
      for (i, row) in enumerate(self.lab):
        # compute the distance between the current L*a*b*
        # color value and the mean of the image
        # d = dist.euclidean(row[0], mean)
        d = np.linalg.norm(row[0] - mean)

        # if the distance is smaller than the current distance,
        # then update the bookkeeping variable
        if d < minDist[0]:
          minDist = (d, i)
      # return the name of the color with the smallest distance
      cnt_colour = self.colorNames[minDist[1]]

      # TODO associate contours to cards

      # then draw the contours and the name of the shape on the image
      # cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)

      (x,y),radius = cv2.minEnclosingCircle(c)
      center = (int(x),int(y))
      radius = int(radius)
      cv_image = cv2.circle(cv_image,center,radius,(0,255,0),2)

      # cv2.putText(cv_image, "%s: %s %s"%(cnt_index, cnt_colour, cnt_shape), center, cv2.FONT_HERSHEY_SIMPLEX,
      #   0.5, (255, 255, 255), 2)

      cnt_dict[cnt_index] = (cnt_colour, cnt_shape, center, radius)

    card_list = cards_from_contours(cnt_dict)

    for card in card_list:
      cv2.putText(cv_image, str(card), tuple(card._center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

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

  def __str__(self):
    return "%s %s %s %s"%(self._number, self._color, self._fill, self._shape)

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


class optical_set_card:

  def __init__(self, card, center, radius):
    self._card = card
    self._center = center
    self._radius = radius

  def combine(self, new_center):
    self._card._number += 1
    # weighted average
    self._center = (self._card._number - 1) * np.array(self._center) + new_center
    self._center = self._center / self._card._number

  def __str__(self):
    return self._card.__str__()

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


def cards_from_contours(contour_dict):
  card_list = []

  for cnt in contour_dict.values():
    # check overlap between contour and all existing cards
    contour_center = cnt[2]
    create_standalone_card = True
    for card in card_list:
      if np.linalg.norm(np.array(contour_center) - card._center) < (3 * card._radius):
        # combine card and contour
        create_standalone_card = False
        card.combine(contour_center)
        continue
    if create_standalone_card:
      card_list.append(optical_set_card(set_card(1, cnt[1], 'solid', cnt[0]), cnt[2], cnt[3]))

  return card_list


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