How to clone:
git clone https://github.com/Soft-Flyers-Group/crab_ws --recurse-submodules     

Instructions to run:
hostname -I (Find IP and enter inside NI Lab View) (To recieve loadcell data)

Run in order
ros2 launch crab_control gait_recorder.launch.py bag_name:=experiment_01.bag

Repeat the steps above for every new bag
source install/setup.bash
run python3 plotter.py to graph a bag, and change bagpath inside the python file for different bags

Running camera and imu node
ros2 run crab_control imu_node
ros2 run crab_control camera_node

To view the stream on your laptop

Install gstreamer:
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

Then run: 
gst-launch-1.0 -v udpsrc port=5600 !     "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG, payload=(int)26" !     rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink

Camera Calibration:
ssh -Y crabbot@ip 
source install/setup.bash
ros2 launch crab_control camera_callibration.launch.py