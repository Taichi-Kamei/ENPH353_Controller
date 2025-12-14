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


== Introduction

=== Competition Requirements and Goals

=== Contribution Split

=== ROS Architecture
Shown below is the structure of our ROS nodes and topics. The ROS nodes are in the black box, and the topics are highlighted in grey. Bold arrows indicate the ROS node interactions through topics, while dashed arrows represents local access relationships.
\
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/ROS.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [ROS Node and Topic Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
\
Our robot has 3 main ROS nodes, competition, state machine, and controller GUI nodes. Connecting those nodes are the topics, and we made 5 new topics for debugging purposes. "/processed_img" and "/tape_img" are used for driving, and "/board_mask_img", "/flattened_plate_img", and "/letters_img" are used for clue detection. All of these images are processed inside each state and are internally referenced to the main state_machine node, which then gets published as Image topics.
\
Our CNN is integrated in clue detect state instead of running independently. This is done so we can transition to clue detect state once the robot faces to the clue board, avoiding unexpected PID control behavior from sudden clue detection.
\
=== Finite State Machine Architecture
Finite State Machine (FSM) was implemented for our robot to manage driving in various surface conditions, detecting obstacles and clue boards. The FSM consists of 16 states, and below is the diagram illustrating the transitions between these states based on sensor inputs.
\
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/FSM.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [Finite State Machine Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
\
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
\
== Driving System

=== Filtering valid contours
For the PID driving, extracting the side lines and filtering out any other noises are crucial. 
We realized that the ground to sky ratio in the frame was always constant on a flat surface, so we first crop the raw image and only keep the ground portion. Then, we gray-scale and binarize the cropped image, and find the contour using _cv2.findContours(img_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)_. \
With high enough binarized threshold, we can filter out most of the small contour in dirt road section and other similar surface condition, but we could not eliminate all of them, so we tried filtering out by contour area. \
\
#figure(
  image("images/find_contour.png",width: 70%),
  caption: [Contours in dirt road section after filtering by area]
)
\
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
\
Through multiple layers of filtering, we finally got valid contours like in the image above.
\

=== General PID Algorithm
We generally use contour's area moment, calculate lane center coordinate, and find the error between center coordinate and the center of the frame. However, unlike in the typical line-following, we drive in the middle of the two white lines, so the proportional control require center lane adjustment algorithm for a smooth drive. 

\
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
\
#figure(
  grid(
    columns:2,
    gutter: 1cm,
    image("images/drive_steep_right.jpg", width: 100%),
    image("images/drive_steep_left.jpg", width: 100%)
  ),
  caption: [Steep curve with single line]
)
The light blue dot is the estimated Cx, and it is very close to the frame center (light red). With the same error calculation as above, the error would be way smaller than the required error to fully turn. This is due to the frame center significantly off from the lane center. To solve this issue, we thought of shifting the frame center more to the side like shown the dark red line on the images. The closer the contour is to the bottom of the frame, the more shift required, so we used the Cy value of the area moment, and tested a proportional relationship between Cy and the magnitude of the shift. The following is the adjusted error calculation code:\

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
\
\
It turned out that a single line driving is far more stable than dual-line driving, so we ended up using single line P-control for most of the time, and made the robot move straight while two lines were detected and the error was within certain range.


\
=== Intentional swerving
Our PID algorithm worked too well, and missed clue boards right after the steep curve because the board was never in the camera frame. To solve this problem, we implemented intentional swerving in which the robot would swerve left and right periodically for a specified period. This allowed the robot to cover a wider area and increased the chances of detecting clue boards.
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
\
=== Roundabout
Difficulties we encountered other than the truck detection was the let turn at the entrance and at the exit of the roundabout.\

For the entrance, we made the robot turn left while exiting the "Truck" state so the robot can start drive clock-wise when it enters the "Roundabout" state. \
\
#figure(
    image("images/roundabout_4thclue.png",width: 40%),
    caption: [Using 4th clue board contour area as transition condition]
)
for exiting roundabout, the robot could turn left for most of the time because we made the P-control favor left turning with center-lane shift algorithm in "Roundabout" state. However, there was a decent chance of robot failing to do so, so we created a "Post_Roundabout" state which rotates left for a short time after entering this state. The transition condition from "Roundabout" state to this state is the contour area of the 4th clue board exceeding threshold area.

=== Off-Road
The strategy for this section was:\
\
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
\

  
=== Mountain
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

The problem with driving up the mountain was the sky showing up as a huge contour. Because it is always at side and will have big contour area, we can't filter it out using the same method as before. We masked out the pale blue sky by turning the raw image to HSV, create blue mask, and use _cv2.bitwise_not()_ to only remove the blue.\

== Obstacle Detection

=== Pedestrian & Truck
#figure(
    image("images/pedestrian_detection.png",width: 70%),
    caption: [Pedestrian detection using cv2.absdiff()]
)
Our strategy for pedestrian and truck detection was to stop the robot when entering the state, use _cv2.absdiff()_ to find the difference in two consecutive frames, and start driving if there were no difference for 2 consecutive times. \

The detection method is the same for both states, but transition to each detecting state is different. \
For the pedestrian detection, we detect the red tape using a red mask, enter "Pedestrian" state, stop the robot, and detect for moving object. We cropped the top part of the frame so that the truck at the back wouldn't be considered as a moving pedestrian.\
\
#figure(
    image("images/pre_truck_detection.png",width: 70%),
    caption: [Flat and wide bounding rectangle at the top of the frame]
)
For the truck state, we made a "Pre_Truck" state which transitions to the "Truck" state when there is a wide and flat bounding rectangle at the top as it is shown in the top right side of the image. At the exit of this state, the robot will turn slightly to the right so the camera can capture the truck better.


=== Baby Yoda
The route we used for off-road driving does not interfere with the Baby Yoda's path. Therefore, we did not have to implement any detection and avoidance 

=== Clue Detection transition Algorithm
#figure(
    image("images/homing_board.png",width: 50%),
    caption: [Homing at the board]
)
Our clue detection CNN runs inside the "Clue_Detect" state only when the robot is facing the board and is close enough. By doing so, we can avoid unexpected behavior, and maximize the chance of predicting right clue.
In each driving state, we have a blue board contour detection function which returns true above certain area threshold. When that becomes true, the robot transitions to the "Clue_Detect" state. We use PID and face to the board, run the CNN, and move the robot closer to the board until the CNN function returns a valid letters. Once the letters are detected, the robot sends it to the score tracker, face away from the clue board, and transitions to the next state depending on the clue type. We implemented downtime after the clue detection because the robot sometimes catch the board again and gets stuck in "Clue Detect" state.\

== Clue Detection

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
\
=== Future Improvements
Use imitation learning
== Appendix