import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only

red_threshold = 170
green_threshold = 170
blue_threshold = 150
###### Threshold for Golden Sample 
red_golden_higher_threshold = 230
red_golden_lower_threshold = 110

green_golden_higher_threshold = 220
green_golden_lower_threshold = 110

blue_golden_higher_threshold = 60
blue_golden_lower_threshold = 0
######

rgb_threshold = (red_threshold, green_threshold, blue_threshold)
rgb_golden_threshold = (red_golden_lower_threshold,\
                red_golden_higher_threshold,\
                green_golden_lower_threshold,\
                green_golden_higher_threshold,\
                blue_golden_lower_threshold,\
                blue_golden_higher_threshold)


def color_thresh_navigable(img, rgb_thresh=(0, 0, 0)):
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
    
def color_thresh_obstacle(img, rgb_thresh=(0, 0, 0)):
    return 1 - color_thresh_navigable(img, rgb_thresh)

def color_thresh_golden(img, rgb_thresh=(0, 0, 0, 0, 0, 0)):
#     print(rgb_thresh)
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Boolean value true for the pixels with RGB value in range

    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,0] < rgb_thresh[1]) \
                & (img[:,:,1] > rgb_thresh[2]) \
                & (img[:,:,1] < rgb_thresh[3]) \
                & (img[:,:,2] > rgb_thresh[4]) \
                & (img[:,:,2] < rgb_thresh[5])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
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
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles



    dst_size = 5 
    bottom_offset = 6

    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
#     plt.imshow(warped)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable =  color_thresh_navigable(warped, rgb_thresh=rgb_threshold)
    obstacle = color_thresh_obstacle(warped, rgb_thresh=rgb_threshold)
    rock = color_thresh_golden(warped, rgb_thresh=rgb_golden_threshold)
# 1024 x 768
    Rover.vision_image[:,:,0] = obstacle*255
    Rover.vision_image[:,:,1] = rock*255
    Rover.vision_image[:,:,2] = navigable*255

#     plt.imshow(navigable, cmap='gray')
#     plt.imshow(obstacle, cmap='gray')
#     plt.imshow(rock, cmap='gray')
    
    # 4) Convert thresholded image pixel values to rover-centric coords
    navigable_x , navigable_y = rover_coords(navigable)
    obstacle_x , obstacle_y = rover_coords(obstacle)
    rock_x , rock_y = rover_coords(rock)
    
    # 5) Convert rover-centric pixel values to world coords
    # added -50 bias for wall crawling
    navigable_x_world , navigable_y_world = pix_to_world(navigable_x, navigable_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    obstacle_x_world , obstacle_y_world = pix_to_world(obstacle_x , obstacle_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    rock_x_world , rock_y_world = pix_to_world(rock_x , rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_x, navigable_y)
    
    # Rover.nav_angles = temp_nav_angles 

    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_x, rock_y)
    Rover.obstacle_dists, Rover.obstacle_angles = to_polar_coords(obstacle_x, obstacle_y)


    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    
    return Rover