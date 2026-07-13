How to clone:
git clone https://github.com/Soft-Flyers-Group/crab_ws --recurse-submodules     

Instructions to run:
hostname -I (Find IP and enter inside NI Lab View) (To recieve loadcell data)

Run in order
ros2 launch crab_control gait_recorder.launch.py bag_name:=experiment_01.bag

Repeat the steps above for every new bag
source install/setup.bash
run python3 plotter.py to graph a bag, and change bagpath inside the python file for different bags