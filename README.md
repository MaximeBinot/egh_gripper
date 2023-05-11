# Schunk EGH Service
ROS VERSION: melodic

Implements ROS Services that provide interfaces to the commands of the Schunk EGH gripper.
The IO Link Interface implementation `src/daemon` was taken from the PolyScope urcaps of the gripper.

## Setup
Clone this repository into `catkin_ws_root/src`.
Resolve dependencies with `rosdep` or install them manually. From `package.xml`.
Then build the package with `catkin_make`.

## Example

	rosrun schunk_egh_service gripper_service.py
	rosrun schunk_egh_service gripper_curses_serviceclient.py

The default IOLink ip and port are set to `192.168.1.253:502`.
If your IO Link has a different adress set the environment variables `IO

## Scripts
`scripts/gripper_service.py` launches a rosnode with a EGHSchunkServer that provides an indiviual rospy.Service for each command of the gripper.
Currently only _grip_, _release_, _positioning_ and _acknowledge_ are implemented.
`scripts/gripper_curses_client.py` a simple curses application that calls the service endpoints.

## Implement a new command
To imlpement the interface to a new command, first create the service specification by create a new file
`new_service_name.srv` in `srv/`.
Next, add the file to the `CMakeLists.txt` `add_service_file` call.
The implement the service in `src/service.py` by adding a new `CommandService` instance to the `_services` list.
By convention, the namespaces of the services are the lower case names of the service class.

## Testing
There are currently no automated test yet. But there is a test mode that uses a mocked daemon. The mocked daemon currently only supports the implemented commands that were listed before.

To launch the gripper_service in test mode pass the `--test` flag when running with `rosrun`.
