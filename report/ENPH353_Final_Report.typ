#import "@preview/clean-cnam-template:1.3.0": *

#show: clean-cnam-template.with(
  title: "ENPH 353 Final Report",
  author: "Taichi Kamei, Bowen Yuan",
  class: "ENPH 353",
  subtitle: "2025 Clue Detective Competition",
  main-color: "#228B22",
  code-font: "JetBrains Mono",
  default-font: "New Computer Modern",
  outline-code: block(
    inset: (left: 1.2cm, right: 1.2cm),
    outline(
      title: "Table of Contents",
      depth: 3,
      indent: auto,
    )
  ),
)

#set page(
  margin: 2.5cm
)

#set text(
  lang: "en",
  size: 12pt
)

#show figure.where(
  kind: image
): set figure.caption(position: bottom)

#show ref: it => {
  if it.element.func() == heading {
    link(it.target, it.element.body) // Links to target, displays heading body
  } else {
    it // Default behavior for other elements
  }
}

== Introduction

=== Competition Requirements and Goals
In this project, we designed a fully autonomous "Fizz Detective" robot that can navigate a racecourse in a simulated ROS Gazebo environment while identifying clues to a crime on boards located around the track. There were two main challenges in this competition:
- Drive around the track without violating any traffic rules
- Detect the prescence of clue boards and read the clues

Points were awarded for succesfully identifying the clues to solve the crime. However, points were deducted for traffic rule violations like driving off the road or hitting other vehicles and pedestrians. There were four minutes of simulation time to try and achieve the maximum 57 points with ties being broken by time.

Our goal for the competition was to make it possible for our robot to achieve maximum points. Specifically, this meant we wanted to be able to drive through the entire course without using respawning our robot as that would incur point deductions and restrict our maximum point potential. 
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/ros_map.png", width: 100%),
  // Add a caption using a content block ([...])
  caption: [Map of ROS Gazebo Competition Enviroment],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/clue_board_example_small.png", width: 100%),
  // Add a caption using a content block ([...])
  caption: [Simulated Camera Feed of Clue Board],
  // Add a label for referencing (use a name enclosed in angle brackets)
)

=== Contribution Split
We brainstormed the overall strategy we would follow together. We decided on using PID navigation as it would be simpler to implement than an imitation learning or reinforcement learning system. Taichi developed the robot's PID navigation algorithm and state machine. For detection of clue boards and optical character recognition, we decided to use a simpler system based on masking the specific blue colour of the board and a small convolutional neural network as opposed to using a YOLO based system. Bowen developed the clue board detection and the convolutional neural network for optical character recognition.

=== ROS Architecture
Shown below is the structure of our ROS nodes and topics. The ROS nodes are in the black box, and the topics are highlighted in grey. Bold arrows indicate the ROS node interactions through topics, while dashed arrows represents local access relationships.
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/ROS.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [ROS Node and Topic Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
Our robot has 3 main ROS nodes, competition, state machine, and controller GUI nodes. Connecting those nodes are the topics, and we made 5 new topics for debugging purposes. "/processed_img" and "/tape_img" are used for driving, and "/board_mask_img", "/flattened_plate_img", and "/letters_img" are used for clue detection. All of these images are processed inside each state and are internally referenced to the main state_machine node, which then gets published as Image topics.

Our CNN is integrated in clue detect state instead of running independently. This is done so we can transition to clue detect state once the robot faces to the clue board, avoiding unexpected PID control behavior from sudden clue detection.

=== Finite State Machine Architecture
Finite State Machine (FSM) was implemented for our robot to manage driving in various surface conditions, detecting obstacles and clue boards. The FSM consists of 16 states, and below is the diagram illustrating the transitions between these states based on sensor inputs.
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/FSM.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [Finite State Machine Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
There are multiple state transition using clue board, and those are done by using the clue type detected by our CNN model. In some states, there is a chance of robot missing the clue board, and failing to transition to the desired state. To avoid this, we decided to have backup transition condition if possible. For example, the transition from "Dirt_Road" to "Narrow_Road" can happen either by detecting the clue board or by detecting the contour of the lake.

== Controller GUI

#grid(
  columns: 2,
  gutter: 0.5cm,
  [  
    We developed a controller GUI for monitoring different view types, controlling simulation environment, and launching scripts.\
    \
    The primary reason why we created this GUI was to increase productivity by centralizing all control on one window instead of having multiple terminal tabs and going on Gazebo to stop or reset robot.
    This allowed us to easily test changes on VSCode without getting frustrated from moving cursor all over the screen.\
    \
    GUI Functionalities:
    - View different driving camera feeds (Raw, Contour, Tape)
    - View different clue detection image processing stages\ (Board Mask, Flattened Plate, Letters)
    - Pause/Resume Gazebo simulation
    - Reset robot position in Gazebo
    - Launch State Machine script
    - Launch Score Tracker script
    - Start and stop score tracker timer (Debugging purposes)
  ],
  figure(
    // The image function goes here (no '#' needed inside figure)
    image("images/Controller_GUI.png", width: 80%),
    // Add a caption using a content block ([...])
    caption: [Controller GUI],
    // Add a label for referencing (use a name enclosed in angle brackets)
  )
)

== Driving System

=== Filtering valid contours
For the PID driving, extracting the side lines and filtering out any other noises are crucial. 
We realized that the ground to sky ratio in the frame was always constant on a flat surface, so we first crop the raw image and only keep the ground portion. Then, we gray-scale and binarize the cropped image, and find the contour using _cv2.findContours(img_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)_.

With high enough binarized threshold, we can filter out most of the small contour in dirt road section and other similar surface condition, but we could not eliminate all of them, so we tried filtering out by contour area. 

#figure(
  image("images/find_contour.png",width: 70%),
  caption: [Contours in dirt road section after filtering by area]
)

As it can be seen, undesired contours still remained after the area filtering. Due to the nature of camera FOV, the contour area gets stretched at the bottom. Therefore, some noises can suddenly pop up at the bottom of the frame even if those were initially filtered out. To resolve this issue, we added another filtering layer using _cv2. boundingRect()_, and only keep the bounding rectangles which touch the sides and is not at the middle bottom of the frame.

#figure(

  grid(
  columns: 3,
  gutter: 0.1cm,

  image("images/dirt_road_contour.png", width: 70%),
  image("images/narrow_road_drive.png", width: 70%),
  image("images/dirt_road_steep.png", width: 70%)
),
  caption: [Valid contours],
)

Through multiple layers of filtering, we finally got valid contours like in the image above.

=== General PID Algorithm
We generally use contour's area moment, calculate lane center coordinate, and find the error between center coordinate and the center of the frame. However, unlike in the typical line-following, we drive in the middle of the two white lines, so the proportional control require center lane adjustment algorithm for a smooth drive. 
#figure(
  image("images/drive_2lane.jpg", width: 70%),
  caption: [2 lanes drive with traditional P-control]
)
In this case, we find the area moment of two lines and average the Cx value to get the lane center which is shown as light blue point. We calculate the error from the frame center which is shown as red line, and control the yaw. This works for straight roads with two lines, but fails at a steep curve where there is only one line.
#code(
  raw(block:true, lang:"Python",
    "error = (frame_width / 2.0) - cx

self.state_machine.move.linear.x  = self.linear_speed
self.state_machine.move.angular.z = self.kp * error"
  )
)
#figure(
  grid(
    columns:2,
    gutter: 1cm,
    image("images/drive_steep_right.jpg", width: 100%),
    image("images/drive_steep_left.jpg", width: 100%)
  ),
  caption: [Steep curve with single line]
)
The light blue dot is the estimated Cx, and it is very close to the frame center (light red). With the same error calculation as above, the error would be way smaller than the required error to fully turn. This is due to the frame center significantly off from the lane center. To solve this issue, we thought of shifting the frame center more to the side like shown the dark red line on the images. The closer the contour is to the bottom of the frame, the more shift required, so we used the Cy value of the area moment, and tested a proportional relationship between Cy and the magnitude of the shift. The following is the adjusted error calculation code:
#code(
  raw(block:true, lang:"Python",
    "slope = 2.5\n
if cx <= frame_width * 0.5:
  slope = -1 * slope

center_shift = slope * cy
error = center_shift + (frame_width / 2.0) - cx"
  )
)
The slope value was adjusted in each state through trial and error, which ranged from 2 to 4.

It turned out that a single line driving is far more stable than dual-line driving, so we ended up using single line P-control for most of the time, and made the robot move straight while two lines were detected and the error was within certain range. There were some challenges with seeing clue boards while driving. For details on our attempt to solve these issues see @swerve.

=== Roundabout
Difficulties we encountered other than the truck detection was the let turn at the entrance and at the exit of the roundabout.

For the entrance, we made the robot turn left while exiting the "Truck" state so the robot can start drive clock-wise when it enters the "Roundabout" state. 
#figure(
    image("images/roundabout_4thclue.png",width: 40%),
    caption: [Using 4th clue board contour area as transition condition]
)
for exiting roundabout, the robot could turn left for most of the time because we made the P-control favor left turning with center-lane shift algorithm in "Roundabout" state. However, there was a decent chance of robot failing to do so, so we created a "Post_Roundabout" state which rotates left for a short time after entering this state. The transition condition from "Roundabout" state to this state is the contour area of the 4th clue board exceeding threshold area.

=== Off-Road
The strategy for this section was:
1. Position the robot perpendicular to the pink tape using edge
#figure(
  grid(columns:2,

  image("images/tape_pink_raw.png", width: 100%, height: 4cm, fit: "contain"),
  image("images/tape_edge.png", width: 100%, height: 4cm, fit: "contain"),
  ),
  caption: [Positioning robot perpendicular to the pink tape]
  )
2. Move straight for some period
3. Turn 90 degrees left
4. Move straight and go over the hill
5. Home at the pink tape next to the tunnel with PID
#figure(
    image("images/tape_home_better.png",width: 30%),
    caption: [Homing at the pink tape]
)
6. Move straight until pink contour area is above threshold
7. Position the robot perpendicular to the tape.
#figure(
    image("images/2nd_pink_align.png",width: 50%),
    caption: [Positioning perpendicular to the pink tape]
)
For details on navigating the mountain which ended up being unused, see @mountain

== Obstacle Detection

=== Pedestrian & Truck
#figure(
    image("images/pedestrian_detection.png",width: 70%),
    caption: [Pedestrian detection using cv2.absdiff()]
)
Our strategy for pedestrian and truck detection was to stop the robot when entering the state, use _cv2.absdiff()_ to find the difference in two consecutive frames, and start driving if there were no difference for 2 consecutive times.

The detection method is the same for both states, but transition to each detecting state is different. \
For the pedestrian detection, we detect the red tape using a red mask, enter "Pedestrian" state, stop the robot, and detect for moving object. We cropped the top part of the frame so that the truck at the back wouldn't be considered as a moving pedestrian.
#figure(
    image("images/pre_truck_detection.png",width: 70%),
    caption: [Flat and wide bounding rectangle at the top of the frame]
)
For the truck state, we made a "Pre_Truck" state which transitions to the "Truck" state when there is a wide and flat bounding rectangle at the top as it is shown in the top right side of the image. At the exit of this state, the robot will turn slightly to the right so the camera can capture the truck better.

=== Clue Detection transition Algorithm
#figure(
    image("images/homing_board.png",width: 50%),
    caption: [Homing at the board]
)
Our clue detection CNN runs inside the "Clue_Detect" state only when the robot is facing the board and is close enough. By doing so, we can avoid unexpected behavior, and maximize the chance of predicting right clue.
In each driving state, we have a blue board contour detection function which returns true above certain area threshold. When that becomes true, the robot transitions to the "Clue_Detect" state. We use PID and face to the board, run the CNN, and move the robot closer to the board until the CNN function returns a valid letters. Once the letters are detected, the robot sends it to the score tracker, face away from the clue board, and transitions to the next state depending on the clue type. We implemented downtime after the clue detection because the robot sometimes catch the board again and gets stuck in "Clue Detect" state.

== Clue Detection
=== Plate Extraction
Once we have a clue board in the raw camera feed, we first `create_blue_mask` out of the raw image using `cv2.inRange` (we found the HSV range of [80, 125, 0] to [160, 255, 255] to work the best). We then `extract_board` by taking the largest contour (and bigger than a `min_sign_area`) and then use `cv2.approxPolyDP` to find the four corners of the plate. Using `cv2.findHomography` and `cv2.warpPerspective` we can then perspective transform it to make a flat projection of the clue board. We repeat this process again except masking for white/gray pixels in order to `extract_plate`.
#figure(
  image("images/image_processing.png", width: 80%),
  caption: [Plate Extraction Image Processing Pipeline]
)

=== Letter Extraction
With our plate image, we mask for the blue letters and find the bounding boxes of all the contours using `cv2.boundingRect`. The plates always have the two words different rows so we `group_contours_into_rows` by identifying whether a contour is in the upper or lower cluster. As well, the letters are sometimes too close together horizontally and get placed into one bounding box so we `process_single_row` and split up bounding boxes with outlying widths. The letters are then resized to our OCR CNN model's input dimensions (100, 150) and the input tensor is passed through to identify the clue category and clue value strings. For more details on our CNN model, see @cnn_sum.
#figure(
  image("images/letter_extraction_pipeline.png", width: 100%),
  caption: [Letter Extraction Image Processing Pipeline]
)

=== Optical Character Recognition Convolutional Neural Network
In order to train our CNN, we chose to start our training dataset generation from the plates instead of generating a dataset straight from clean images of the `UbuntuMono-R.tff` font because then artifacts of different letters neighbouring each other would be preserved in the dataset. 

We generated clean plates with random five character alphanumeric strings. To ensure a uniform dataset, we made sure each character appeared on a plate 10 times for a total of 360 clean plates. Then each clean plate was distorted in 20 different ways with a Keras ImageDataGenerator applying random rotations, zooms, and brightness changes, as well as our custom preprocessing function, `mangle`, which added a random noise map, random gaussian blur, and random small perspective transforms. In total we had 200 training images per character for a total of 7200 training images. For more details on the training data generation, see @cnn_data.

For training, we used a 30% validation split. We used an initial learning rate of $1 times 10^(-4)$ for 100 epochs. However, we also had an `EarlyStopping` callback monitoring validation loss with a patience of 10 epochs and a `ReduceLROnPlateau` callback which would reduce the learning right by a factor of 0.99 when validation loss plateaued for 3 epochs. The CNN was training for 87/100 epochs and restored the best weights from epoch 77.  The final categorical cross entropy loss 0.1740 and the validation loss was 0.1847

== Conclusion


=== Competition Result
#figure(
  image("images/7th_clue_detect.png", width: 80%),
  caption: [7th Clue Detected]
)
#figure(
  image("images/comp_result.png", width: 80%),
  caption: [Unofficial but during competition result]
)

Scored 38 points though unrecorded.
We were the only team with traditional PID control that has reached to the tunnel section and captured the 7th clue board.  
As a team who did the PID control with the original robot, we think it is very hard to implement PID control robot to finish the course under tight time. This is mainly due to the code complexity required and unavoidable uncertainty at the off-road section arising from 2nd pink tape homing, 2nd pink tape alignment. All the teams that went beyond the tunnel were either using imitation learning or drone PID. 

=== Future Improvements
Use imitation learning
== Appendix
=== Intentional swerving <swerve>
Our PID algorithm followed the outer curver of the road too well, and missed clue boards right after the steep curve because the board was never in the camera frame. We realized this too late to simply change the FOV of our camera as that would mean reworking the entire driving system. To solve this problem, we implemented intentional swerving in which the robot would swerve left and right periodically for a specified period. This allowed the robot to cover a wider area and increased the chances of detecting clue boards.
This swerving was implemented in a rather simple way by using a counter that gets incremented every time the state's run function gets called, and changing the "slope" value periodically for $"mod"2 = 0$.
#code(
  raw(block: true, lang: "python", 
  "slope = 1.4\n
if self.count <= 95 and self.count >= 85 and self.count % 2 == 0:
  slope = 3.7\n
if cx <= frame_width * 0.5:
  slope = -1 * slope\n
center_shift = slope * cy
error = center_shift + (frame_width / 2.0) - cx

self.state_machine.move.linear.x  = self.linear_speed
self.state_machine.move.angular.z = self.kp * error")
)

This technique was used in "Post_Crosswalk" and "Post_Roundabout" states.
=== Mountain <mountain>
#grid(
  columns: 2,

  figure(
    image("images/mountain_before.png",width: 70%),
    caption: [Sky showing up as a huge contour]
  ),

  figure(
    image("images/mountain_sky_black.png",width: 70%),
    caption: [Sky masked out]
  )
)

The problem with driving up the mountain was the sky showing up as a huge contour. Because it is always at side and will have big contour area, we can't filter it out using the same method as before. We masked out the pale blue sky by turning the raw image to HSV, create blue mask, and use _cv2.bitwise_not()_ to only remove the blue.
=== CNN Summary <cnn_sum>
Here is the final CNN design we used for the competition and some details about it's performance.
#table(
// caption: "CNN Model Architecture Summary",
columns: (10cm, 3cm, 3cm),
align: (left, left, right),
inset: 2pt,
stroke: (left: black, right: black, top: black, bottom: black),
[*Layer (type)*], [*Output Shape*], [*Param \#*],
[conv2d (Conv2D)], [148 × 98 × 32], [320],
[max_pooling2d (MaxPooling2D)], [74 × 49 × 32], [0],
[dropout (Dropout)], [74 × 49 × 32], [0],
[conv2d_1 (Conv2D)], [72 × 47 × 64], [18,496],
[max_pooling2d_1 (MaxPooling2D)], [36 × 23 × 64], [0],
[dropout_1 (Dropout)], [36 × 23 × 64], [0],
[conv2d_2 (Conv2D)], [34 × 21 × 128], [73,856],
[max_pooling2d_2 (MaxPooling2D)], [17 × 10 × 128], [0],
[dropout_2 (Dropout)], [17 × 10 × 128], [0],
[flatten (Flatten)], [21760], [0],
[dense (Dense)], [256], [5,570,816],
[dropout_3 (Dropout)], [256], [0],
[dense_1 (Dense)], [36], [9,252],
)

Total parameters: 5,672,740 (21.64 MB)\
Trainable parameters: 5,672,740 (21.64 MB)\
Non-trainable parameters: 0 (0.00 B)

Best Epoch:\
`Epoch 77/100
       [1m315/315[0m [32m━━━━━━━━━━━━━━━━━━━━[0m[37m[0m [1m5s[0m 11ms/step - acc: 0.9673 - loss: 0.1740 - val_acc: 0.9671 - val_loss: 0.1847 - learning_rate: 9.2274e-05`

#grid(
  columns: 2,

  figure(
    image("images/loss.png",width: 100%),
    caption: [CNN Model Categorical Cross Entropy Loss]
  ),

  figure(
    image("images/acc.png",width: 100%),
    caption: [CNN Model Accuracy]
  ),
)

#figure(
    image("images/confusion.png",width: 90%),
    caption: [Confusion Matrix]
  )

=== CNN Dataset Generation <cnn_data>