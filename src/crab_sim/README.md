Added package to show flipper movement based on ros2bag data

Steps to run

- New terminal
source install/setup.bash
ros2 launch crab_sim flippersim.launch.py

- New terminal
source install/setup.bash
ros2 run crab_sim sim_pub

- New terminal
ros2 bag play "bag folder name"

- Watch the flipper movee!