# randomwalker
Tutorial code on ROS basics: publishers, subscribers, and services.

## Introduction
randomwalker is a small, toy robot simulator to practice implementing ROS publishers, subscribers, and services. There are four components to randomwalker.

The **world** is a 2D grid. Each location on the grid has a **score** associated with it, ranging from -10 to 10. As your robot visits each location, it adds the location's score to its total score. Try to race around and get a high score!

The world is already implemented for you. Instead, you will implement the **map server**, which provides two services:

1. `get_bounds`: When called, returns the number of rows and columns in the world.
2. `get_score`: Given a row and column number, returns the score of that location in the world.

The **robot** can't see the scores of any location until it travels there. It listens for movement commands on the `move_command` topic. As it moves to each location, it asks the map server what the score is for that location, and adds the score to its total. It also prints that out to the terminal.

The **teleop** allows you to send movement commands to the robot over the `move_command` topic.

## How to write a publisher, subscriber, and service

The ROS tutorials provide a minimal example of how to write publishers, subscribers, and services. See [Writing a Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) and [Writing a Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).

You can get more detailed information in the full rospy documentation on [publishers / subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) and [services](http://wiki.ros.org/rospy/Overview/Services)

## Getting started with randomwalker
Create a catkin workspace if you don't already have one:

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

Clone this repository to your catkin workspace:

```
cd catkin_ws/src
git clone git@github.com:hcrlab/randomwalker.git
```

Build the code so that you get the generated code for the services:

```
cd catkin_ws
catkin_make
```

Now load the setup.bash for this workspace:
```
source catkin_ws/devel/setup.bash
```

You will need to do this for **every terminal window** you create.

## What you need to do

It's recommended to edit the files in this order: teleop.py, mapserver.py, robot.py. You don't need to edit world.py.

### teleop.py

1. Make this into a node called 'teleop'
2. Create a publisher for the 'move_command' topic, of type std_msgs/String.
3. Don't forget import the String message.
4. Publish the command verbatim.
5. Do you need to call rospy.spin()?

Test your changes by opening another terminal window. In this new terminal window, don't forget to run `source catkin_ws/devel/setup.bash`. In the first terminal window, run your code with `rosrun randomwalker teleop.py`. In the second terminal window, run `rostopic echo /move_command`. If you type "up" in the first terminal window, then it should be echoed in the second.

### mapserver.py

1. Make this into a node called 'map_server' (in main())
2. Add a rospy.spin() to the end of main()
3. Create a service called 'get_bounds', with the implementation in
   `self._get_bounds`
4. Don't forget to import GetBounds.
5. Implement `self._get_bounds` by returning a GetBoundsResponse with the number
   of rows and columns in the world.
6. Create a service called 'get_score', with the implementation in
   `self._get_score`
7. Implement `self._get_score`, by calling self._world.get_score. Get the row
   and col being queried from the request object.
8. (Optional) Check that the queried location is in bounds, and if not, raise a
   rospy.ServiceException.

Test your changes by opening another terminal window. In this new terminal window, don't forget to run `source catkin_ws/devel/setup.bash`. In the first terminal window, run your code with `rosrun randomwalker mapserver.py`. In the second terminal window, run `rosservice call get_bounds`. You should get:

```
num_rows: 10
num_cols: 10
```

Now try running `rosservice call get_score 5 5`. You should get `score: 9`. If you implemented bounds checking, you can try something like `rosservice call get_score 5 11`, and you should get an error message.

### robot.py

1. Make this into a node called 'robot' (in main())
2. Add a rospy.spin() to the end of main()
3. Create a subscriber for the 'move_command' topic, of type std_msgs/String.
   Make `self._handle_move` the callback.
4. When the movement command is received, the robot will call `self._sense`,
   which calls `self._get_score`. `_get_score` makes a service call to
   'get_score'. Implement the service call.

Test your changes by opening two new terminal windows. In each new terminal window, don't forget to run `source catkin_ws/devel/setup.bash`. In the first terminal window, run the map server with `rosrun randomwalker mapserver.py`. In the second terminal window, run the robot with  `rosrun randomwalker robot.py`. In the third window, run the teleop with `rosrun randomwalker teleop.py`. The best setup is if you can see the teleop and robot windows at the same time.

You should be able to type "up", "down", "left", or "right" in the teleop. In the robot window, you will see the robot `(R)` move around, revealing more of the map as you move to new locations. The robot's score will go up or down depending on what location you move to. And it should refuse to go off the edge.

## If you need help
Ask someone! We are happy to help. If you are working on your own you can email HCR Lab. If you are in the lab, then just grab somebody.

If you **really** are stuck and can't ask anyone, then a sample solution is available in the [finished](https://github.com/hcrlab/randomwalker/tree/finished) branch.
