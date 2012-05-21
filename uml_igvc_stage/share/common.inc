define sicklms laser
(
  range_max 12.0
  fov 180.0
  samples 180
  color "black"
  size [ 0.25 0.25 0.1 ]
)

define robot position
(
  size [0.75 0.75 0.5]
  gui_nose 1
  drive "diff"
  name "robot" 
  color "red"
  sicklms(pose [ 0.25 0.000 0 0.000 ])
)

define floorplan model
(
  color "gray30"
  boundary 1

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1

  gui_move 0
)

resolution 0.02
interval_sim 100

