# oakd_pro.yaml:

RGB i_low_bandwidth: true — compresses the RGB image with H.26x before sending over USB/network. 
Works well since RGB is a standard video format (NV12/YUV).

Stereo i_low_bandwidth: false — sends raw depth data. 
This is necessary because depth images use 16UC1 encoding (16-bit unsigned integer per pixel), 
which the H.26x video encoder can't handle (error: Arrived frame type (14) is not either NV12 or YUV400p).

Raw stereo uses more bandwidth, but at 640x400 resolution it's manageable.


# oakd_pro.yaml:
scp ~/turtlebot4_ws/src/tb4_perception/config/oakd_pro.yaml ubuntu@turtlebot4:/tmp/oakd_pro.yaml
ssh ubuntu@turtlebot4
cat /tmp/oakd_pro.yaml
sudo cp /tmp/oakd_pro.yaml /opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_pro.yaml

# oakd.launch.py:
scp ~/turtlebot4_ws/src/tb4_perception/launch/oakd.launch.py ubuntu@turtlebot4:/tmp/oakd.launch.py
ssh ubuntu@turtlebot4
cat /tmp/oakd.launch.py
sudo cp /tmp/oakd.launch.py /opt/ros/jazzy/share/turtlebot4_bringup/launch/oakd.launch.py
