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
Shown below is the structure of our ROS nodes and topics. The ROS nodes are in the black box, and the topics are highlighted in grey. Bold arrows indicate the ROS node interacations through topics, while dashed arrows represents local access relantionships.
\
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/ROS.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [ROS Node and Topic Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
\
Our robot has 3 main ROS nodes, the competition, state machine, and controller GUI nodes. Conneting those nodes are the topics, and we made 5 new topics for debugging purposes alongside the existing topics. "/processed_img" and "/tape_img" are used for driving, and "/board_mask_img", "/flattened_plate_img", and "/letters_img" are used for clue detection.

Our CNN is integrated in clue detect state instead of running indepently, so we can transition to clue detect state once the robot faces to the clue board. This allows robot to make predictable movement, avoiding unexpected PID control behavior from sudden clue detection.


\
=== Finite State Machine Architecture
Finite State Machine (FSM) was implemented on our robot to manage driving in various surface conditions, detecting obstacles and clue boards. The FSM consists of 16 states, and below is the diagram illustrating the transitions between these states based on sensor inputs.
\
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/FSM.pdf", width: 110%),
  // Add a caption using a content block ([...])
  caption: [Finite State Machine Diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
\
There are multiple state transition that uses clue board, and those are done by using the top word of the clue board which gets detected by our CNN model. In some states, there are chances of robot missing the clue board, and failing to transition to the desired state. To avoid that from happening, we decided to have backup state transition conditions if possible. For example, the transition from "Dirt_Road" to "Narrow_Road" can be transitioned either by detecting the clue board or by detecting the countour of the lake. This allowed 

== Controller GUI


== Clue Detection


== Driving System

=== General PID Algorithm

=== Roundabout

=== Off-Road Section

=== Mountain


== Obstacle Detection

=== Pedestrian

=== Truck

=== Baby Yoda


== Conclusion

=== Competition Result

=== Future Improvements

== Appendix




= Example code
== Shaft 2 Calculations

Net driving force on the timing belt pulley is given by:
$
  F_N = T_C / (D_C / 2) = 0.521 "lbf"
$
Bending force on C is given by:
$
  F_C = 1.5 F_N = 0.866 "lbf"
$
Since z components of timing belt cancles out, we only consider x component of $F_C$.

$
F_(C x) = F_C cos(phi) = 0.819 "lbf"
$
Where $phi$ is the angle between the belt and horizontal plane, calculated in above section.\
\
#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/2nd_shaft_FBD.jpeg", width: 70%),
  // Add a caption using a content block ([...])
  caption: [FBD of shaft 2],
  // Add a label for referencing (use a name enclosed in angle brackets)
)
\
Point A and D are the bushings, B is the worm gear, and C is the pulley for the timing belt.\
\
Known forces from above calculations are:
#align(center,
  grid(
    columns: 2,
    gutter: 1cm,

    $F_(B x) = 3.28 "lbf" \
    F_(B y) = 2.67 "lbf" \
    F_(B z) = 12.5 "lbf" \ $,

    $F_(C x) = 0.819 "lbf"
    $
  )
)
\
Forces on A and D are calcualated as follows:
\
#align(left,
$Sigma F_x = 0 :$
) $
  -F_(A x) + F_(B x) - F_(C x) - F_(D x) &= 0 \
  F_(A x) + F_(D x) &= F_(B x) - F_(C x) \
  F_(A x) &= F_(B x) - F_(C x) - F_(D x) \
  $

#align(left,
$Sigma F_y = 0 :$
)
  $
  F_(A y) - F_(B y) &= 0 \
  F_(A y) &= F_(B y)
  \
  $

#align(left,
$Sigma F_z = 0 :$
)
  $
  F_(A z) - F_(B z) + F_(D z) &= 0 \
  F_(A z) + F_(D z) &= F_(B z) \
  F_(A z) &= F_(B z) - F_(D z)
  $

#align(left,
$Sigma M_(A x) = 0 :$
)
  $
  -L_(A B)F_(B z) + L_(A D)F_(D z) &= 0 \
  F_(D z) &= L_(A B)/L_(A D)F_(B z) \ 
  $

#align(left,
$Sigma M_(A z) = 0 :$
)
  $
  -L_(A B)F_(B x) + L_(A C)F_(C x) +  L_(A D)F_(D x) &= 0 \
  F_(D x) &= (L_(A B)F_(B x) - L_(A C)F_(C x))/L_(A D) \ 
  $
\
Using the known forces, we get following forces on A and D:

#align(center,
  grid(
    columns: 2,
    gutter: 1cm,  
    $F_(A x) = 3.01 "lbf" \
    F_(A y) = 2.67 "lbf" \
    F_(A z) = 11.7 "lbf" \
    $,
    $F_(D x) = -0.553 "lbf" \
    F_(D y) =  0.00 "lbf" \
    F_(D z) = 0.812 "lbf" \
    $
  )
)
\

#align(center,
  grid(
    columns: 2,
    gutter: 1cm,

    figure(
      table(
        columns: 3,
        stroke: 1pt + black,
        inset: 4pt,
          [],[$"V"_("horizontal") "(lbf)"$],  [$"V"_("vertical") "(lbf)"$],
          [A],[-3.01],  [11.7],
          [B],[0.267],  [-0.812],
          [C],[-0.553], [-0.812],
          [D],[0.00],   [0.00]
      ),
      caption: [Shear Forces],
    ),

    figure(
      table(
        columns: 4,
        stroke: 1pt + black,
        inset: 4pt,
          [], [$"M"_("horizontal") "(lbf·in)"$], [$"M"_("vertical") "(lbf·in)"$], [$"M"_("total") "(lbf·in)"$],
          [A],[0.00],[0.00],[0.00],
          [B],[-3.01],[11.7],[12.1],
          [C],[-1.23],[0.406],[1.30],
          [D],[0.00],[0.00],[0.00]
      ),
      caption: [Bending Moments],
    )
  )
)

#figure(
  // The image function goes here (no '#' needed inside figure)
  image("images/2nd_shaft_shear_moment.jpeg", width: 70%),
  // Add a caption using a content block ([...])
  caption: [Shear and bending moment diagram],
  // Add a label for referencing (use a name enclosed in angle brackets)
) <fig:2nd_shaft_shear_moment>
\
#align(center,
  block(width: auto,
    grid(
      columns: 2,
      gutter: 1cm,

      figure(
        table(
          columns: 2,
          stroke: 1pt + black,
          inset: 4pt,
            [], [$"T (lbf·in)"$],
            [A], [0.00],
            [B], [0.521],
            [C], [-0.521],
            [D], [0.00]
        ),
        caption: [Torque],
      ),

      figure(
        image("images/2nd_shaft_torque.jpeg", width: 70%),
        caption: [Torque diagram of 2nd shaft],
      ),
    )
  )
)

\
#figure(
table(
  columns: 3,
  stroke: 1pt + black,
  inset: 4pt,
    [], [$"M"_("total") "(lbf·in)"$], [$"T (lbf·in)"$],
    [A], [0.00],  [0.00],
    [B], [12.1],  [0.521],
    [C], [1.30],  [-0.521],
    [D], [0.00],  [0.00]
),
caption: [Summary of Moments and Torques],
)

The constraint we have is the diameter sizes for the worm gear and the pulley. The mateiral is suitable for the shaft if the minimum diameter calculation at each location is below the allowable diameter determined by those components' bore diameter.

From Mott eqn 12-24,

$
D_min = [
  (32N) / pi sqrt((k_t M / S_n')^2 + (3/4) (T / S_y)^2)
]^(1/3)
$
\
For the material choice, we want to use an affordable, and easy to machine. Therefore, we will use Aluminum for the shaft material. \
From Appendix B II, we choose Aluminum 2014 O for its high ductility, decent strength, and cheap cost of about \$1 per inch.
\
#align(center,
  figure(
    table(
      columns: 2,
      stroke: 1pt + black,
      inset: 4pt,
        [$S_u$], [27 ksi],
        [$S_y$], [14 ksi],
        [$S_(n)$], [13 ksi],
    ),
    caption: [Material Properties of Aluminum 2014 O],
  )
)
\

$S_n' = 10.608 "ksi"$
The modified endurance strength is same as Shaft 1 calculations. \
$K_t = 2.5$ as sharp fillet is used for the shaft shoulders.\
$N = 2.0$ is chosen for our design factor since aluminum is a ductile material and the design factor is in the range of $1.5 < N < 2.5$. 
\

Substituting the values into the minimum diameter based on moment and torque, we get:
$
D_(min, i) = [
  (64) / pi sqrt((2.5 M_("total", i) / 10608)^2 + (3/4) (T_i / 14000)^2)
]^(1/3) "for" i = A, B, C, D
$

Minimum diameter based on shear:
$ D_(min "shear") = sqrt((2.95 K_t V_i N)/(s'_n)) "for" i = A, B, C, D $
\
Using eqn (6.4.25), (6.4.26) and table 2,3, we calculate the minimum shaft diameter at each location: \
\
#align(center,
  figure(
    table(
      columns: 4,
      stroke: 1pt + black,
      inset: 4pt,
        [], [$D_min "(in)"$],[$D_(min "shear") "(in)"$],[$D_"Allowable" "(in)"$],
        [$D_A$], [0.00],[0.130],[0.186],
        [$D_B$], [0.362],[0.0345],[0.75],
        [$D_C$], [0.172],[0.0370],[0.25],
        [$D_D$], [0.00],[0.00], [0.186]
    ),
    caption: [Minimum and Allowable Shaft Diameters],
  )
)

\
The minimum shaft diameter at each location is well below the allowable shaft diameter determined by the components. 
Thus, Aluminum 2014 O is a suitable material for the 2nd shaft.
\