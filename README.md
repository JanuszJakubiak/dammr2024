# Configure environment

## Initial setup of the workspace

Open a terminal (from app menu or `Ctrl-Alt-t`)

```
mkdir -p dammr2024_ws/src
cd dammr2024_ws/
colcon build
```
The lines above create a workspace folder with the `src` subfolder. Then `colcon build` is used to generate other necessary subfolders and files.


## Source workspace setup files

To configure ROS2 CLI and, among others, fill the completion lists, source setup files

```
source install/local_setup.bash && source install/setup.bash
```

## Create a package

Create a python package with a "Hello World" example, build it and "register" the new package and executables
```
cd src
# ros2 pkg create --build-type ament_python --node-name <my_node> <my_package>
ros2 pkg create --build-type ament_python --node-name hello_world dammr_python
cd ..
colcon build --packages-select dammr_python
source install/local_setup.bash
```

And now run the example
```
ros2 run dammr_python hello_world
```

## Adding a new script

You may add a new python script as a ROS2 node by placing it in  the `<workspace>/src/dammr_python/dammr_python/` folder.

After placing the script in the folder, make it executable
```
chmod a+x <path>/<script_name>
```
and add it to the `entry_points` section of `setup.py`

Finally, `cd` to the workspace folder and run `colcon build` 


## Remarks

Remember to always run `colcon build` in the workspace folder (not home or source code folders).
It is adviced to have a separate terminal to build and run nodes, and a second to navigate and edit source files.


# ROS2 CLI

## Topics

Check the list of available topics
```
ros2 topic list
```

Choose a topic from the list, inspect it and see published data
```
ros2 topic info <topicname>
ros2 topic hz <topicname>
ros2 topic bw <topicname>
ros2 topic echo <topicname>
```

Inspect the message type shown by `topic info`
```
ros2 interface show <msgtype>
```
(but you may also find message description via simple google search in a web browser)


## Nodes


Similarly you may inspect  available nodes with:
```
ros2 node list
ros2 node info <node
```

## Inspecting the system

Displaying log messages
```
ros2 run rqt_console rqt_console
```

Displaying system structure
```
ros2 run rqt_graph rqt_graph
```

# Pioneer robot

The examples below may be run as standalone scripts, outside a package

## Simple subscriber to read LIDAR data

```
#!/usr/bin/env python

# run: python3 subscriber_scan.py 5  (where 5 is robot number)
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.ranges)
    if scan:
       print("  Scan min: %f  front: %f" % ( min(scan),scan[len(scan)//2]))


def main(args=None):
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] <robot_number>\n')
        sys.exit(1)
    rclpy.init()
    nr=sys.argv[1]

    node = Node('listener')

    # Subscribe topics and bind with callback functions
    node.create_subscription(LaserScan, f"/pioneer{nr}/scan", callback_scan, 10)

    # spin(node) simply keeps python from exiting until this node is stopped
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Observe the output for different obstacles around the robot.
You may try to visualize the data with `matplotlib`.
Design a formula to detect obstacles to be avoided (you may use a circular or a rectangular emergency region)


## Publisher

```
#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/pioneer5/cmd_vel', 10)
        self.get_logger().info('Velocity Publisher has been started.')

    def publish_velocity(self,v,w):
        velocity_command = Twist()
        velocity_command.linear.x = v  # Replace this with your desired linear velocity
        velocity_command.angular.z = w  # Replace this with your desired angular velocity

        self.publisher_.publish(velocity_command)
        self.get_logger().info('Published velocity command: linear={}, angular={}'.format(
            velocity_command.linear.x, velocity_command.angular.z))

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    time.sleep(8)
    velocity_publisher.get_logger().info('Start')
    try:
        velocity_publisher.publish_velocity(0.0,0.5)
        time.sleep(2)
        velocity_publisher.publish_velocity(0.0,-0.5)
        time.sleep(2)
        velocity_publisher.publish_velocity(0.0,0.0) 	
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```


# UWB tranceiver
We will use Qorvo (former Decawave) ultra-wideband (UWB) [DWM1001-DEV](https://www.qorvo.com/products/p/DWM1001-DEV) development boards for real time location system (RTLS).
Details and manuals may be found in the [documentation page](https://www.qorvo.com/products/p/DWM1001-DEV#documents).


## CLI of the DWM1001 module

```
minicom --device /dev/ttyACM0
dwm> ?
```
Please check some of the command outputs, in particular listing configuration and get measurements from the module.

## Python script to communicate with the module

1. Write a script that connects to the module, runs a selected command, receives the output from the module and prints it to the screen reformatted.

   Hint: an example can be found in [`dwm`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/tree/dwm) branch, file [`dwm_simple.py`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/blob/dwm/dwm_simple.py) (switch to active branch with `git checkout dwm` in the repository)

2. Modify the script to run a command to print position in loop, print the coordinates on the screen

   Hint: an example can be found in [`dwm`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/tree/dwm) branch, file [`dwm_lep.py`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/blob/dwm/dwm_lep.py)


## ROS2 publisher

1. Create a package with the `hello_world` example, run the example to check if everything is correct

2. Create a script of a string publisher following the [ROS2 tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node), run it and verify via `topic echo` that the script works correctly

   Hint: solutions can be found in the [`publisher`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/tree/publisher) branch (`git checkout publisher` in the repository)


3. Transform the position reading script from the previous task to a ROS2 publisher. Begin with publishing the position as a string. Then consider what message type would fit better.

4. Consider how the node can be extended (using other topics? fault detection? configuration parameters?)

An example of a solution may be found in the [`solution_string`](https://github.com/JanuszJakubiak/dammr2024-ros2workshop/tree/solution_string) branch.

# Appendix

## usefull ROS2 message types

```
# std_msgs/msg/String
string data

# geometry_msgs/msg/Vector3
float64 x
float64 y
float64 z


# geometry_msgs/msg/Twist
Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z


# geometry_msgs/msg/Vector3Stamped
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Vector3 vector
	float64 x
	float64 y
	float64 z

# sensor_msgs/msg/LaserScan
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```


## DWM1001 commands

Initial output after connection/reset of the module
```
DWM1001 TWR Real Time Location System

 Copyright :  2016-2017 LEAPS and Decawave
 License   :  Please visit https://decawave.com/dwm1001_license
 Compiled  :  Nov 29 2017 13:35:02

 Help      :  ? or help
```

Output of the help command
```
Usage: <command> [arg0] [arg1] ...
Build-in commands:

** Command group: Base **
?: this help
help: this help
quit: quit

** Command group: GPIO **
gc: GPIO clear
gg: GPIO get
gs: GPIO set
gt: GPIO toggle

** Command group: SYS **
f: Show free memory on the heap
ps: Show running threads
pms: Show PM tasks
reset: Reboot the system
si: System info
ut: Show device uptime
frst: Factory reset

** Command group: SENS **
twi: General purpose TWI read
aid: Read ACC device ID
av: Read ACC values

** Command group: LE **
les: Show meas. and pos.
lec: Show meas. and pos. in CSV
lep: Show pos. in CSV

** Command group: UWBMAC **
nmg: Get node mode
nmp: Set UWB mode to passive
nmo: Set UWB mode to off
nma: Set mode to AN
nmi: Set mode to ANI
nmt: Set mode to TN
nmtl: Set mode to TN-LP
bpc: Toggle BW/TxPWR comp
la: Show AN list
stg: Get stats
stc: Clear stats

** Command group: API **
tlx: Send TLV frame1234
clk: W-cfg clk pins out
aurs: Set upd rate
aurg: Get upd rate
apg: Get pos
aps: Set pos
acas: Set anchor config
acts: Set tag config

** Tips **
Press Enter to repeat the last command
```


