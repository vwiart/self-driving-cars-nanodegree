import numpy as np
import image
import camera

class Perspective(object):
    
    def __init__(self, src, dst):
        self.src = Rectangle(Point(562, 474),
                             Point(724, 474),
                             Point(1005, 663),
                             Point(298, 663))
        self.dst = Rectangle(Point(320, 0),
                             Point(1005, 0),
                             Point(1005, 663),
                             Point(320, 663))
class Point(object):
    """A point is a couple of coordinate."""

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_tuple(self):
        """Transform a `Point` in a tuple of coordinate."""
        return (self.x, self.y)


class Rectangle(object):
    """This object represents a rectangle."""

    def __init__(self, top_left, top_right, bottom_right, bottom_left):
        self.top_left = top_left
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right
        self.top_right = top_right

    def to_array(self):
        """Convert a rectangle to a list of coordinates."""
        return np.float32([self.top_left.to_tuple(),
                           self.bottom_left.to_tuple(),
                           self.bottom_right.to_tuple(),
                           self.top_right.to_tuple(),])
