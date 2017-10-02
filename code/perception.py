import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def maxmin_color_thresh(img, min_rgb_thresh, max_rgb_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    mid_thresh = (img[:,:,0] > min_rgb_thresh[0]) \
                & (img[:,:,0] < max_rgb_thresh[0]) \
                & (img[:,:,1] > min_rgb_thresh[1]) \
                & (img[:,:,1] < max_rgb_thresh[1]) \
                & (img[:,:,2] > min_rgb_thresh[2]) \
                & (img[:,:,2] < max_rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[mid_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst_size = 5 
    bottom_offset = 6
    s1_x, s1_y, d1_x, d1_y = 14, 140, image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset
    s2_x, s2_y, d2_x, d2_y = 301 ,140, image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset
    s3_x, s3_y, d3_x, d3_y = 200, 96, image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset
    s4_x, s4_y, d4_x, d4_y = 118, 96, image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset

    source = np.float32([[s1_x, s1_y], 
                         [s2_x, s2_y], 
                         [s3_x, s3_y], 
                         [s4_x, s4_y]])
    destination = np.float32([[d1_x, d1_y], 
                              [d2_x, d2_y], 
                              [d3_x, d3_y], 
                              [d4_x, d4_y]])
    
    # 2) Apply perspective transform
    warped_img = perspect_transform(image, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    
    warped_rock_bin_img = maxmin_color_thresh(warped_img, min_rgb_thresh=(110, 110, 10), max_rgb_thresh=(270, 240, 70))
    warped_terrain_bin_img = color_thresh(warped_img) - warped_rock_bin_img
    warped_obstacles_bin_img = np.ones_like(warped_terrain_bin_img) - warped_terrain_bin_img - warped_rock_bin_img
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = warped_obstacles_bin_img*255
    Rover.vision_image[:,:,1] = warped_rock_bin_img*255
    Rover.vision_image[:,:,2] = warped_terrain_bin_img*255
    # 5) Convert map image pixel values to rover-centric coords
    rover_obstacles_centric_bin_img = rover_coords(warped_obstacles_bin_img)
    rover_terrain_centric_bin_img = rover_coords(warped_terrain_bin_img)
    rover_rock_centric_bin_img = rover_coords(warped_rock_bin_img)
    
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    obstacle_x_world, obstacle_y_world = pix_to_world(rover_obstacles_centric_bin_img[0], 
                                    rover_obstacles_centric_bin_img[1], 
                                    Rover.pos[0], 
                                    Rover.pos[1], 
                                    Rover.yaw, 
                                    Rover.worldmap.shape[0], 
                                    scale)
    navigable_x_world, navigable_y_world = pix_to_world(rover_terrain_centric_bin_img[0], 
                                    rover_terrain_centric_bin_img[1], 
                                    Rover.pos[0], 
                                    Rover.pos[1], 
                                    Rover.yaw, 
                                    Rover.worldmap.shape[0], 
                                    scale)
    rock_x_world, rock_y_world = pix_to_world(rover_rock_centric_bin_img[0], 
                                    rover_rock_centric_bin_img[1], 
                                    Rover.pos[0], 
                                    Rover.pos[1], 
                                    Rover.yaw, 
                                    Rover.worldmap.shape[0], 
                                    scale)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(obstacle_y_world, obstacle_y_world)
    return Rover