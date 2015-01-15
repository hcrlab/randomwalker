#!/usr/bin/env python

from world import World

"""
### Steps for this file

1. Make this into a node called 'robot' (in main())
2. Add a rospy.spin() to the end of main()
3. Create a subscriber for the 'move_command' topic, of type std_msgs/String.
   Make `self._handle_move` the callback.
4. When the movement command is received, the robot will call `self._sense`,
   which calls `self._get_score`. `_get_score` makes a service call to
   'get_score'. Implement the service call.

### Helpful resources

- [ROS tutorial on publisher / subscribers](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- [ROS tutorial on services](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
- [Official rospy documentation on publisher / subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
- [Official rospy documentation on services](http://wiki.ros.org/rospy/Overview/Services)
"""

class Robot(object):
    """Represents a robot.

    A robot has its own representation of the world. Initially, it does not 
    know the scores for all the squares in the grid. As it explores, it can get
    the score for the grid it's currently on.
    """
    def __init__(self, world):
        # TODO: Create a subscriber for the 'move_command' topic.
        # Make self._handle_move the callback.
        self._world = world
        self._row = self._world.num_rows() / 2
        self._col = self._world.num_cols() / 2
        self._score = 0
        self._sense()

    def _get_score(self):
        """Gets the score for the robot's current location."""
        # TODO: Call the 'get_score' service. Remember to wait for it first.
        # TODO: Be sure you're returning an integer. When you call a service,
        # the result is actually a response object. What do you need to do to
        # get the integer out?
        return None

    def _sense(self):
        """Senses the score for the current location.

        Also updates the robot's model of the world, as well as its score
        counter.
        """
        self._world.set_robot_location(self._row, self._col)
        score = self._get_score()
        self._world.set_score(self._row, self._col, score)
        self._score += score
        rospy.loginfo('\n%s', self._world)
        rospy.loginfo('Score: %d', self._score)

    def _handle_move(self, msg):
        """Callback for movement commands.

        After every movement, calls self._sense to update the robot's world
        view. If the movement command is invalid, then the robot doesn't move or
        sense.
        """
        command = msg.data
        success = False
        if command == 'up':
            success = self._move_up()
        elif command == 'down':
            success = self._move_down()
        elif command == 'left':
            success = self._move_left()
        elif command == 'right':
            success = self._move_right()
        else:
            raise ValueError('Invalid command: {}'.format(command))
        if not success:
            return
        self._sense()

    def _move_up(self):
        if self._row == 0:
            rospy.logerr('Can\'t go up.')
            return False
        self._row -= 1
        return True

    def _move_down(self):
        if self._row == self._world.num_rows() - 1:
            rospy.logerr('Can\'t go down.')
            return False
        self._row += 1
        return True

    def _move_left(self):
        if self._col == 0:
            rospy.logerr('Can\'t go left.')
            return False
        self._col -= 1
        return True

    def _move_right(self):
        if self._col == self._world.num_cols() - 1:
            rospy.logerr('Can\'t go right.')
            return False
        self._col += 1
        return True

def get_world():
    # TODO: Call the 'get_bounds' service (defined in mapserver.py). Don't
    # for get to wait for it first.

    # TODO: Fill out the constructor with the number of rows and columns
    # returned from the 'get_bounds' service.
    return World(None, None)

def main():
    # TODO: Make this program into a ROS node called 'robot' using init_node.
    world = get_world()
    robot = Robot(world)
    # TODO: Call rospy.spin()

if __name__ == '__main__':
    main()
