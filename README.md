source /opt/ros/jazzy/setup.bash

source ~/assignment1_wk/install/setup.bash

colcon build --packages-select avg_brightness_node

source install/setup.bash

ros2 run image_tools cam2image

ros2 run image_tools showimage

ros2 run image_tools cam2image --ros-args --params-file config/cam2image.yaml

ros2 run avg_brightness_node avg_brightness_node

ros2 run cam2image_vm2ros cam2image --ros-args -p device_id:=0 -p show_camera:=true
