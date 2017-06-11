## Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/distortion_correction_sample.png "Undistorted"
[image2]: ./output_images/distortion_correction.png "Road Transformed"
[image3]: ./output_images/combining_thresholds.png "Combining Threshold"
[image4]: ./output_images/thresholds_hls_channel.png "Threshold HLS"
[image5]: ./output_images/pipeline_threshold.png "Binary Example"
[image6]: ./output_images/perspective_transform.png "Warp Example"
[image7]: ./output_images/window_fitting.png "Fit Visual"
[image8]: ./output_images/equation_radius_of_curvature.png "Output"
[image9]: ./output_images/lane_detection.png "Output"
[video1]: ./output_videos/project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "[./Advanced-Lane-Lines.ipynb.ipynb](./Advanced-Lane-Lines.ipynb.ipynb)".

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result:

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

Combining Threshold

![alt text][image3]

We have 2 cases (color thresholding steps at 6th code cell of the IPython notebook):

* More shadow:  used color threshold with L channel.
* More light: I used color threshold with S channel

![alt text][image4]

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at 8th code cell of the IPython notebook).  Here's an example of my output for this step.


![alt text][image5]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in the 10th code cell of the IPython notebook.  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32(
    [[(img_size[0] / 2 - 85), img_size[1] / 2 + 100],
    [                      0, img_size[1]],
    [            img_size[0], img_size[1]],
    [ (img_size[0] / 2 + 85), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])
```

This resulted in the following source and destination points:

| Source        | Destination   |
|:-------------:|:-------------:|
| 555, 460      | 320, 0        |
| 0, 720        | 320, 720      |
| 1280, 720     | 960, 720      |
| 725, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image6]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:

![alt text][image7]

I used some steps (14th code cell):

* First find the two starting positions for the left and right lane at the bottom of frame 
* Go through each layer looking for positions for the next left and right lane using past position as a reference

To optimize the result, I used width of lane to find the correct positions (line 35-48 and 88-96).

* Predict two positions for the left or right lane using width of lane (start with value is 470px) if we cannot get both of left and right lane positions in 1 frame.
```python
    if (l_max > 0 and r_max == 0): # Cannot find r_center
        l_center = l_max - offset
        r_center = l_center + len_of_lane + 20
    elif (l_max == 0 and r_max > 0): # Cannot find l_center
        r_center += r_max - offset + 20
        l_center = r_center - len_of_lane
    elif (l_max > 0 and r_max > 0): # Found l_center and r_center
        l_center = l_max - offset
        r_center += r_max - offset + 20
        if (abs(r_center - l_center - len_of_lane) > offset_frame):
            if (np.average(l_conv) >= np.average(r_conv)): # Find trustly point
                r_center = l_center + len_of_lane
            else:
                l_center = r_center - len_of_lane
```
I used history information of past frame for current frame to make the lane line more smoothly (line 51-58 and 98-104).
```python
    if len(window_centroids_pre) > 0:
        l_center_pre = window_centroids_pre[0][0]
        r_center_pre = window_centroids_pre[0][1]

        if abs(l_center_pre-l_center) > offset_frame:
            l_center = l_center_pre
        if abs(r_center_pre-r_center) > offset_frame:
            r_center = r_center_pre
```
#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in the 17th and 22th code cell of the IPython notebook, using function `cal_line_fit()`.

* Our curvature line of lane is f(y)=Ay**2 + By + C
* Using function `np.polyfit()` to get [A, B, C]
```python
line_fit = np.polyfit(ploty, linex, 2)
```
* Calculate all `f(y)`:
```python
line_fitx = line_fit[0]*ploty**2 + line_fit[1]*ploty + line_fit[2]
```
* Fit new polynomials to x,y in world space
```python
xm_per_pix=3.7/500
ym_per_pix=20/720
line_fit_cr = np.polyfit(ploty*ym_per_pix, linex*xm_per_pix, 2)
```
* Measuring curvature

![alt text][image8]

```python
line_curverad = ((1 + (2*line_fit_cr[0]*y_eval*ym_per_pix + line_fit_cr[1])**2)**1.5) / np.absolute(2*line_fit_cr[0])
```

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in the 22th code cell of the IPython notebook in the function `drawing_lane()`.

Step by step:

* Combining threshold gray and color image to make lane line more clearly
* Warped image to convert from birdview to top-down view
* Use Gaussian Blur to reduce noise
* Sliding window search
* Measure curvature
* Draw lane

Here is an example of my result on a test image:

![alt text][image9]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

##### 1. Techniques:
I used all techniques which provided in all lesson of Project Advanced Lane Finding.
To improve the result, I had some techniques:

* Color Threshold with HLS image using LS channel: Combine Threshold with L channel and S channel depend on the current image has more shadow or more light. To reduce noise I applied GaussianBlur techniques after perspective transform step.
* Perspective Transform: I increased region for detection, so I can cover all cases during processing test video because the lane is not always in the center of video.
* Sliding Window: With cases cannot find the first left or right point, I used lenght of lane for prediction. Using history imformation is a good idea to make the lane line more smoothly. And decrease region in image for lane detetion is a good idea to remove some noise from other cars, trees ... But it make my pipeline cannot detect multiple lane line.

##### 2. Cases maybe failed:

* Have some noise in center of lane
* Have too much shadow
* Lane line is not clear
* The curvature of lane is too big as harder_challenge_video.mp4
* The width of lane is always changed and has not a stable value.

##### 3. Improved:

* Maybe combining with other techniques such as: Canny-Houghline will improve my result.
* Other idea is using deep neural network for predict lane line. Maybe I will try this idea when I have more free time.
