NUM_ROWS = 10
NUM_COLS = 10

"""
This file needs no modification for the tutorial.
"""

class World(object):
    """Represents the world the robot is in.

    The simulated world is represented as a 2D array. Each location has a score
    in the range [-10, 10].
    """
    def __init__(self, num_rows, num_cols):
        """Initializes the world to all 0s."""
        self._num_rows = num_rows
        self._num_cols = num_cols
        self._robot_row = -1 # The location of the simulated robot.
        self._robot_col = -1
        self._map = [
            ['?' for c in range(num_cols)]
            for r in range(num_rows)
        ]

    def get_score(self, row, col):
        """Retrive the score at a particular row and column."""
        if row >= self._num_rows or col >= self._num_cols:
            raise ValueError('Row or column is out of bounds.')
        return self._map[row][col]

    def set_score(self, row, col, val):
        """Set the score at a particular row and column."""
        if row >= self._num_rows or col >= self._num_cols:
            raise ValueError('Row or column is out of bounds.')
        self._map[row][col] = val

    def set_robot_location(self, row, col):
        """Sets the robot's location at a particular row and column."""
        self._robot_row = row
        self._robot_col = col
    
    def randomize(self, rng):
        """Fills the map with random numbers in the range [-10, 10]."""
        self._map = [
            [rng.randint(-10, 10) for c in range(self._num_cols)]
            for r in range(self._num_rows)
        ]

    def num_rows(self):
        return self._num_rows

    def num_cols(self):
        return self._num_cols

    def __str__(self):
        """Print the map in a human-readable format."""
        map_str_rows = [
            ' '.join([
                '{:>7}'.format(
                    '{}{}'.format(
                        score,
                        ' (R)' if r == self._robot_row and c == self._robot_col
                        else ''
                    )
                )
                for c, score in enumerate(row)
            ])
            for r, row in enumerate(self._map)
        ]
        return '\n'.join(map_str_rows)

