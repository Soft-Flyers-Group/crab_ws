How to clone:
git clone https://github.com/Soft-Flyers-Group/crab_ws --recurse-submodules     

Instructions to run:
hostname -I (Find IP and enter inside NI Lab View) (To recieve loadcell data)

Run in order
ros2 bag record -o [BAGNAME].bag/load_cell_data /servo/encoder_data /servo/position_data
ros2 run crab_control load_cell_node
ros2 run crab_control servo_node
ros2 run crab_control servo_controller

Repeat the steps above for every new bag

run python3 plotter.py to graph a bag, and change bagpath inside the python file for different bags