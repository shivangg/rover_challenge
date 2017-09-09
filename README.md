## Project: Search and Sample Return
<!-- ### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"

* Test out the functions in the Jupyter Notebook provided

* Test the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.

* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  
 -->
[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[test_video]: https://www.youtube.com/watch?v=XLubcUM_pXQ
[overview]: ./misc/overview.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[mapping]: ./misc/1vjivc.gif

<!-- ## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

--- -->
### Writeup / README

### Video of This Project
#### Checkout the [test video][test_video] of this Rover challenge on my YouTube playlist.

### Notebook Analysis
#### 1. Ran the functions provided in the well designed Jupyter Notebook that helped even afterwards to rapidly design solutions for the autonomous driving. Defined functions for:
1. Navigation thresholding
2. Obstacle thresholding
3. Rock Sample thresholding

![mapping overview][mapping]

#### 2. The `process_image()` function is populated  with the appropriate analysis steps to identify the pixels corresponding to 

* Navigable terrain using `color_thresh_navigable`

* Obstacle (inversion of the navigable terrain) using the `obstacle_thresh_obstacle`

* Golden Rock sample using the `color_thresh_golden` to find the pixels within the RGB range of the golden rocks.

Finally, map them onto the ground truth map of the world. The output images from the  `process_image()` are used to create video output using the `moviepy` functions. 


![overview image][overview]

### Autonomous Navigation and Mapping

<!-- #### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were. -->

#### 1. The perception_step is derived from the `process_image` function in the Notebook. It gave a 30% improvement when I considered only a cropped out middle portion of the vision\_image rather than the whole image.

```python
    
h, w = navigable.shape[:2]
delta = 50
region_of_interest = navigable[h - delta: h,\
 						int(w/2) - delta : int(w/2) + delta ]

navigable_x , navigable_y = rover_coords(region_of_interest)

```

### The decisions made by the rover for autonomous driving:
#### 1. Right wall following by setting an offset to the steering that depends on the mean of nav_angles

```python

steer_offset = -15
Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi + steer_offset) \
					, -15, 15)

```

#### 2.Detect if the rover is stuck by checking if the `Rover` state variable `Rover.stuck` if above the defined threshold `Rover.stuck_time_threshold` and start turning right to get out of stuck state.

```python
if Rover.vel < 0.1 and Rover.vel > -1.0:
    Rover.stuck_time_threshold += 1
    Rover.throttle = Rover.throttle_set
    if Rover.stuck_time_threshold > 100:
        Rover.stuck = True
    if Rover.stuck:
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

```
#### 3. Fine tuned the constants for for better turning and faster controlling of the Rover

```python

self.stop_forward = 150 # Initiate stopping sooner to prevent bumping into obstacle
self.go_forward = 400 # Threshold to go forward again
self.throttle_set = 0.6 # Increased throttle setting when accelerating for faster navigation

```



#### 4. Added code in the  `supporting_functions.py` for debugging purposes.

```python
cv2.putText(map_add,"  Nav_angle len: "+str((len(Rover.nav_angles))), (0, 155), \
            cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
cv2.putText(map_add,"  Mode: "+str(Rover.mode), (0, 145), \
            cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
```



#### 2. Launching in autonomous mode the rover can navigate and map autonomously. It can be improved by using:

1. HSV thresholding.

2. Using A* for path planning.

3. Better way to manage the state of the `Rover`.

4. PID tuning for the constants `threshold_set`, `go_forward`, `stop_forward`.

#### Specifications to reproduce the results:

* Resolution: `1024 x 768`

* Graphis quality: Good

* `Mapping: 99.9%`

* `Fidelity: 75.1%`

* Time Taken: 420 seconds @ 20 `Frames per second`


#### The Unity simulator can be downloaded for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip) and [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip)

