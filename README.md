This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

# Team

Name|Email
:-----:|:-----:
Kevin Sepehri|kevinsepehri@gmail.com
Lajos Kamocsay|panka.nospam@gmail.com
Mike Challis|gardenermike@gmail.com
Rafael Barreto|rafaelbarretorb@gmail.com

# ROS Nodes
## Waypoint Updater
The purpose of this node is to publish a fixed number of waypoints (40) ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. 

[/ros/src/waypoint_updater/waypoint_updater.py](/ros/src/waypoint_updater/waypoint_updater.py)

### Subscribes to

`/current_pose`: The current position of the car.

`/base_waypoints`: One time load of waypoints from the whole track.

`/current_velocity`: The current velocity of the car.

`/traffic_waypoint`: Waypoint index of the closest red traffic light.

Not subscribing to obstacle waypoint as it's not yet part of the project

### Publishes

`/final_waypoints`: Total number of waypoints are based on LOOKAHEAD_WPS variable. If the car is approaching a red light it reduces these velocities to come to a stop by the stop line.
## Drive By Wire (DBW)
This node subscribes to various topics and controllers to provide appropriate throttle, brake, and steering commands. 

[ros/src/twist_controller/dbw_node.py](ros/src/twist_controller/dbw_node.py)

### Subscribes to

`/twist_cmd`: Twist commands that describe linear and angular velocities

`/current_velocity`: The current velocity of the car

`/vehicle/dbw_enabled`: Switching between manual and DBW control

`/tl_detector_ready`: Boolean that returns true once the traffic light detector is ready. If the detector is not ready the car brakes and sleeps.

### Publishes

`/vehicle/throttle_cmd`: Throttle commands.

`/vehicle/brake_cmd`: Brake commands.

`/vehicle/steering_cmd`: Steering commands.

## Traffic Light Detection
This node uses Keras MobileNet to detect traffic lights and publishes the upcoming light waypoint along with a boolean for knowing if the model is ready. 

[ros/src/tl_detector/tl_detector.py](ros/src/tl_detector/tl_detector.py)

### Subscribes to

`/current_pose`: The current position of the car.

`/base_waypoints`: One time load of waypoints from the whole track.

`/vehicle/traffic_lights`: Location of the traffic light in 3D map space.

`/image_color`: Color image provided by camera.

### Publishes

`/tl_detector_ready`: Boolean to notify other nodes that the keras model is loaded.

`/traffic_waypoint`: Waypoint index of the closest red traffic light.

### Training
The model was trained with a subset of the Bosch data and simulator images. The data is then augmented and evaluated. The model gets 95.7% accuracy on the combined simulator and bosch data, with 99.7% accuracy on the simulator training data.

[traffic-light-detection/train.py](traffic-light-detection/train.py)

```python
image_size = (224, 224, 3)
batch_size = 16
num_classes = 4
epochs = 96


base_model = MobileNet(
  alpha=0.25,          # adjust down to make model smaller/faster by reducing filter count
  depth_multiplier=1,  # adjust down to make model smaller/faster by reducing resolution per layer
  weights='imagenet',
  #weights=None,
  include_top=False,
  #classes=num_classes,
  input_shape=image_size
)
```

It also augments the data randomly

```python
# augment data
# rotate up to 2 degrees
image = preprocess.random_rotation(image, 2, row_axis=0, col_axis=1, channel_axis=2)
# randomly shift up to 20%
image = preprocess.random_shift(image, 0.2, 0.2, row_axis=0, col_axis=1, channel_axis=2)
# randomly zoom in up to 20%
image = preprocess.random_zoom(image, (0.8, 0.8), row_axis=0, col_axis=1, channel_axis=2)
#adjust brightness
image = preprocess.random_brightness(image, (0.8, 1.2))
# randomly flip horizontally
if np.random.random() > 0.5:
   image = preprocess.flip_axis(image, 1)
```

## Performance Tuning
We ran into performance issues between the simulator and ROS so we tried the following optimizations:
* Reducing lookahead waypoints from 100 to 40
* Only send images to the model if the car is 100 to 25 waypoints from the light

# Install

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
