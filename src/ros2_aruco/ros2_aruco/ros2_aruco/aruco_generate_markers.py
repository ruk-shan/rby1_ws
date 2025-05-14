#!/usr/bin/env python3

"""
Enhanced ArUco Marker Generator

This script generates ArUco marker images with various customization options:
- Multiple marker IDs
- Different dictionary types
- Custom output size
- Border options
- Output directory specification
"""

import cv2
import numpy as np
import os
import argparse

# Dictionary of available ArUco dictionaries in OpenCV
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
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

def generate_aruco_marker(dictionary, marker_id, size=300, border=1, white_border=0, output_dir=".", show=False):
    """
    Generate an ArUco marker and save it to a file.
    
    Args:
        dictionary: OpenCV ArUco dictionary object
        marker_id: ID of the marker to generate
        size: Size of the output marker image in pixels
        border: Border size (relative to marker size)
        white_border: Size of additional white border in pixels
        output_dir: Directory to save the marker image
        show: Whether to display the marker image
    
    Returns:
        Path to the generated marker image
    """
    # Create the marker image
    marker_img = np.zeros((size, size, 1), dtype="uint8")
    marker_img = dictionary.generateImageMarker(marker_id, size, marker_img, border)
    
    # Add white border if requested
    if white_border > 0:
        # Create a larger white image
        bordered_size = size + 2 * white_border
        bordered_img = np.ones((bordered_size, bordered_size, 1), dtype="uint8") * 255
        
        # Place the marker in the center
        bordered_img[white_border:white_border+size, white_border:white_border+size] = marker_img
        marker_img = bordered_img
    
    # Create output filename
    output_filename = os.path.join(output_dir, f"aruco_marker_{marker_id}.png")
    
    # Save the marker image
    cv2.imwrite(output_filename, marker_img)
    
    # Display the marker if requested
    if show:
        window_name = f"ArUco Marker {marker_id}"
        cv2.imshow(window_name, marker_img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)
    
    return output_filename

def generate_aruco_grid(dictionary, marker_ids, grid_size=(2, 2), marker_size=200, 
                        margin=20, white_border=0, output_dir=".", show=False):
    """
    Generate a grid of ArUco markers and save it to a file.
    
    Args:
        dictionary: OpenCV ArUco dictionary object
        marker_ids: List of marker IDs to include in the grid
        grid_size: Tuple (rows, cols) specifying the grid dimensions
        marker_size: Size of each marker in pixels
        margin: Margin between markers in pixels
        white_border: Size of white border around each marker in pixels
        output_dir: Directory to save the grid image
        show: Whether to display the grid image
    
    Returns:
        Path to the generated grid image
    """
    rows, cols = grid_size
    
    # Adjust marker size if white border is added
    effective_marker_size = marker_size
    if white_border > 0:
        effective_marker_size = marker_size + 2 * white_border
    
    # Calculate the size of the grid image
    grid_width = cols * effective_marker_size + (cols + 1) * margin
    grid_height = rows * effective_marker_size + (rows + 1) * margin
    
    # Create a white image for the grid
    grid_img = np.ones((grid_height, grid_width), dtype="uint8") * 255
    
    # Generate and place markers in the grid
    marker_count = 0
    for r in range(rows):
        for c in range(cols):
            if marker_count >= len(marker_ids):
                break
                
            marker_id = marker_ids[marker_count]
            marker_count += 1
            
            # Generate marker image
            marker_img = np.zeros((marker_size, marker_size, 1), dtype="uint8")
            marker_img = dictionary.generateImageMarker(marker_id, marker_size, marker_img, 1)
            
            # Add white border if requested
            if white_border > 0:
                # Create a larger white image
                bordered_size = marker_size + 2 * white_border
                bordered_img = np.ones((bordered_size, bordered_size, 1), dtype="uint8") * 255
                
                # Place the marker in the center
                bordered_img[white_border:white_border+marker_size, white_border:white_border+marker_size] = marker_img
                marker_img = bordered_img
                effective_size = bordered_size
            else:
                effective_size = marker_size
            
            # Calculate position in the grid
            x = margin + c * (effective_size + margin)
            y = margin + r * (effective_size + margin)
            
            # Place marker in the grid - ensure dimensions match
            # Extract the single channel from marker_img to match grid_img dimensions
            grid_img[y:y+effective_size, x:x+effective_size] = marker_img.reshape(effective_size, effective_size)
            
            # Add marker ID text below the marker
            cv2.putText(grid_img, f"ID: {marker_id}", (x, y + effective_size + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)
    
    # Create output filename
    ids_str = "-".join(str(id) for id in marker_ids[:marker_count])
    output_filename = os.path.join(output_dir, f"aruco_grid_{ids_str}.png")
    
    # Save the grid image
    cv2.imwrite(output_filename, grid_img)
    
    # Display the grid if requested
    if show:
        window_name = "ArUco Marker Grid"
        cv2.imshow(window_name, grid_img)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)
    
    return output_filename

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ArUco Marker Generator')
    parser.add_argument('--dict', type=str, default='DICT_4X4_50',
                        choices=ARUCO_DICT.keys(), help='ArUco dictionary to use')
    parser.add_argument('--ids', type=int, nargs='+', default=[0],
                        help='Marker IDs to generate (space-separated)')
    parser.add_argument('--size', type=int, default=300,
                        help='Size of the marker image in pixels')
    parser.add_argument('--border', type=int, default=1,
                        help='Border size (relative to marker size)')
    parser.add_argument('--white-border', type=int, default=50,
                        help='Size of white border around markers in pixels')
    parser.add_argument('--grid', action='store_true',
                        help='Generate a grid of markers instead of individual files')
    parser.add_argument('--grid-rows', type=int, default=2,
                        help='Number of rows in the grid')
    parser.add_argument('--grid-cols', type=int, default=2,
                        help='Number of columns in the grid')
    parser.add_argument('--margin', type=int, default=20,
                        help='Margin between markers in the grid (pixels)')
    parser.add_argument('--output-dir', type=str, default='.',
                        help='Directory to save the marker images')
    parser.add_argument('--show', action='store_true',
                        help='Display the generated markers')
    args = parser.parse_args()
    
    # Check if the specified dictionary is valid
    if args.dict not in ARUCO_DICT:
        print(f"Error: Invalid ArUco dictionary: {args.dict}")
        print(f"Valid options: {', '.join(ARUCO_DICT.keys())}")
        return
    
    # Create output directory if it doesn't exist
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    # Get the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args.dict])
    
    # Generate markers
    if args.grid:
        # Generate a grid of markers
        output_file = generate_aruco_grid(
            aruco_dict, args.ids, 
            grid_size=(args.grid_rows, args.grid_cols),
            marker_size=args.size, margin=args.margin,
            white_border=args.white_border,
            output_dir=args.output_dir, show=args.show
        )
        print(f"Generated marker grid: {output_file}")
    else:
        # Generate individual markers
        for marker_id in args.ids:
            output_file = generate_aruco_marker(
                aruco_dict, marker_id, 
                size=args.size, border=args.border,
                white_border=args.white_border,
                output_dir=args.output_dir, show=args.show
            )
            print(f"Generated marker {marker_id}: {output_file}")

if __name__ == "__main__":
    main()
