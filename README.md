# PickRobotRosSerial

Code for an Arduino AtMega2560 attached to a simple portal robot with step motors on each axis and vacuum gripper.

This reads a robot command containing a velocity command for each axis and gripper on/off command from the serial interface that was send via the code in the [ros2_serial_example repo](https://github.com/osrf/ros2_serial_example) (or compatible, i.e., cobs encoded message with cdr serialized command).
The message description can be found at [ro45 portalrobot interfaces repo](https://github.com/matl-hsk/ro45_portalrobot_interfaces).


## Building
1. Check out the code including submodule
2. Open the folder in platformIO
3. Run build and upload to the atmega board with the motors and gripper attached to the pins mentioned in the pin_definitions.h header.
4. Run ros2_serial_example with config file adapted to the serial port and robot command messages.
5. Publish a robot command to the ros2 network.
