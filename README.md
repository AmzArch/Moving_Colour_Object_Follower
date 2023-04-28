# Moving_Colour_Object_Follower
Follow a moving object (Green Red or Blue)

To use this package, you need to have ROS2 installed. If you haven't installed ROS2 yet, you can follow the instructions on ROS2 official [website](https://docs.ros.org/en/galactic/Installation.html) (This was developed on ROS2 Galactic).

There are two packages blob_following_bot and blob_interfaces. The blob_interfaces package contains the necessary files to enable a custom message which is used to publish and subscribe the location and area of all detected blobs. 

You can clone this repository to your local machine and build the package using colcon

## Usage

To run the object detection and following package, you need to:

Launch the webcam driver node using (Publishes data from the webcam to ROS2 topic):

```
ros2 launch blob_following_bot image_publisher
```

Then we launch the moving blob detector using the following command (Subscribes to the /image topic and then detects and publishes the list of all moving blobs in the images to /detected_blob topic:
```
ros2 launch blob_following_bot moving_blob_detector
```

Then finally we can launch the node to follow these blobs (Send  commands to turtlebot to follow the blobs):
```
ros2 launch blob_following_bot follow_blob
```

By default, the package is set to follow green blobs, but you can easily change this by modifying line 12 in [follow_blob.py](https://github.com/AmzArch/Moving_Colour_Object_Follower/blob/main/src/blob_following_bot/blob_following_bot/follow_blob.py).


## Testing
To test the package, you can either simulate it in Gazebo or run it on a Turtlebot3 Lite.

## Simulation
To simulate the package in Gazebo, you need to launch the simulation using:
```
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
```

More information on launching the simulation can be found [here](https://ubuntu.com/blog/simulate-the-turtlebot3)
