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


## ROS Packages

### `blob_interfaces`: 
This package contains the necessary components to enable the custom ROS Message BlobsList and Blob used in our main package.

### `blob_following_bot`:
This package has three nodes:

1. `image_publisher`: This node is used to publish the webcam view from laptop/pc to the ROS topic `/image` to simulate the object being tracked. With a real topic, we can directly subscribe to the `/color/preview/image` topic.

2. `moving_blob_detector`: This node subscribes to the `/image` topic (In real turtlebot switch to `/color/preview/image` topic) and then applies HSV filters and background subtraction techniques to isolate only the moving blobs in the image of Red, Blue, and Green color. It then publishes them to the `/detected_blobs` topic.

    *Note*: `blob_detector` is an old file that just publishes the position of the Red, Green, and Blue blobs and does not check if they are moving.

3. `follow_blob`: This node subscribes to the `/detected_blobs` topic and then, according to the color set in line 12 of the code (green in this case), follows the largest blob of the given color by tracking the centroid of the blob on the screen and giving proportional velocity depending on the area and position of the blob.
