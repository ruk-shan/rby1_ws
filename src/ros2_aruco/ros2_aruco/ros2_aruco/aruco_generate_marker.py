"""
Script for generating Aruco marker images.

Author: Nathan Sprague
Version: 10/26/2020
"""

from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import sys

# Project: ArUco Marker Generator
# Python version: 3.8

desired_aruco_dictionary = "DICT_4X4_50"
aruco_marker_id = 1
output_filename = "marker_id1.png"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

def main():
  """
  Main method of the program.
  """
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      desired_aruco_dictionary))
    sys.exit(0)
     
  # Load the ArUco dictionary
  this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
   
  # Allocate memory for the ArUco marker
  # We create a 300x300x1 grayscale image, but you can use any dimensions you desire.
  print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(
    desired_aruco_dictionary, aruco_marker_id))
     
  # Create the ArUco marker
  this_marker = np.zeros((300, 300, 1), dtype="uint8")
  this_marker = this_aruco_dictionary.generateImageMarker(aruco_marker_id, 300, this_marker, 1)
   
  # Save the ArUco tag to the current directory
  cv2.imwrite(output_filename, this_marker)
  cv2.imshow("ArUco Marker", this_marker)
  cv2.waitKey(0)
   
if __name__ == '__main__':
  print(__doc__)
  main()
