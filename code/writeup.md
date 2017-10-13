# Search and Sample Return: Writeup Report
### Jonathan Sullivan
## process_image()
I modified the process_image() to demonstrate your analysis and how you created a worldmap. I defined source and destination points for a perspective transform. The source point were picked using visual inspection and estimated incremental change. bottom_offset is used to tell the robot how close to itself can it see and dst_size is used to scale down the image to the desired size.


```python
def process_image(img):
    ...
    dst_size = 5 
    ...
    bottom_offset = 6
    ...
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
    ...

```

Next I applied the transform on the image.


```python
def process_image(img):
    ...
    warped_img = perspect_transform(img, source, destination)
    ...
```

Next, I applied the color threshold to identify navigable terrain/obstacles/rock 
samples. I first chose to Identify the rock sample with a max-min threshold. 
Then used an online image color picker online to find the RBG value in the 
terrain in the shade. Next I used a one-way threshold to determine what could be
terrain or rock. I then used element-wise subtraction to remove the rock sample 
values. Next I said if something is not terrain or sample rock it obstacle.



```python
def process_image(img):
    ...
    warped_rock_bin_img = maxmin_color_thresh(warped_img, min_rgb_thresh=(110, 110, 10), max_rgb_thresh=(270, 240, 70))
    warped_terrain_bin_img = color_thresh(warped_img, rgb_thresh=(150, 150, 150)) - warped_rock_bin_img
    warped_obstacles_bin_img = np.ones_like(warped_terrain_bin_img) - warped_terrain_bin_img - warped_rock_bin_img
    ...
```
Next I converted thresholded image pixel values to rover-centric coords.


```python
def process_image(img):
    ...
    rover_obstacles_centric_bin_img = rover_coords(warped_obstacles_bin_img)
    rover_terrain_centric_bin_img = rover_coords(warped_terrain_bin_img)
    rover_rock_centric_bin_img = rover_coords(warped_rock_bin_img)
    ...
```

Next I converted rover-centric pixel values to world coords


```python
def process_image(img):
    ...
    scale = 10
    obstacle_x_world, obstacle_y_world = pix_to_world(rover_obstacles_centric_bin_img[0], 
                                    rover_obstacles_centric_bin_img[1], 
                                    data.xpos[data.count], 
                                    data.ypos[data.count], 
                                    data.yaw[data.count], 
                                    data.worldmap.shape[0], 
                                    scale)
    rock_x_world, rock_y_world = pix_to_world(rover_terrain_centric_bin_img[0], 
                                    rover_terrain_centric_bin_img[1], 
                                    data.xpos[data.count], 
                                    data.ypos[data.count], 
                                    data.yaw[data.count], 
                                    data.worldmap.shape[0], 
                                    scale)
    navigable_x_world, navigable_y_world = pix_to_world(rover_rock_centric_bin_img[0], 
                                    rover_rock_centric_bin_img[1], 
                                    data.xpos[data.count], 
                                    data.ypos[data.count], 
                                    data.yaw[data.count], 
                                    data.worldmap.shape[0], 
                                    scale)
    ...
```
Finally I update displayed worldmap.


```python
def process_image(img):
    ...
    data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    data.worldmap[rock_y_world, rock_x_world, 1] += 1
    data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    ...
```

## perception_step() 
For the perception_step() I basically implemented the code from `process_image()` in the simulator enviroment.

```python
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
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(rover_terrain_centric_bin_img[0], 
                                                        rover_terrain_centric_bin_img[1])
    return Rover
```

## decision_step() 
For the decision_step() I only changed the behavior of when to start and stopped by setting 
a threshold values `stop_forward` and `go_forward` in driver_rover.py.

```python
self.stop_forward = 400 # Threshold to initiate stopping
self.go_forward = 500 # Threshold to go forward again
```

Next I changed the robot turned when it started and stoped to be the clipped 
average of all possiblities. 
```python
Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
```
I think It would be worth exploring in future senarios to add some type of destructive or contructive randomness to the turning process.