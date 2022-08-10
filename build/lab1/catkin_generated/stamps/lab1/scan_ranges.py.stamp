r"""
This module defines the `ScanRanges` class, which subscribes to the `/scan`
topic, from which it reads a `LaserScan` message and publishes the closest and
farthest points to `/closest_point` and `/farthest_point`, respectively as
`Float64` values.
"""

# ROS Imports
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import rospy as ros

# Useful Imports
import numpy as np


class ScanRanges:
    def __init__(self) -> None:
        # Initialize our ROS node
        ros.init_node('scan_ranges', anonymous=True)

        # Define our subscribers
        self.sub = ros.Subscriber("scan", LaserScan, self.read_and_relay, queue_size=10)

        # Define our publishers
        self.pub_closest = ros.Publisher("closest_point", Float64, queue_size=10)
        self.pub_farthest = ros.Publisher("farthest_point", Float64, queue_size=10)

        # It's always best to add a queue_size because there's nothing more annoying
        # than running out of memory because of an unbounded buffer and a slow function.
        # After all, it's always adjustable

    def read_and_relay(self, scan: LaserScan) -> None:
        """
        This method acts as our callback for the subscriber to `/scan`. It
        processes the data in the provided scan and publishes the appropriate
        data in each publisher.
        """
        
        # Because `scan.ranges` is known to be of type `float32[]`, we'll
        # just pass that in as the `dtype` (data type) for better static
        # type checking.
        distances = np.array(scan.ranges, dtype=np.float32)

        # Remember, always read the message class documentation. the `ranges`
        # member of `LaserScan` contains all the data measured, but does not
        # filter invalid data. This data has to be filtered by us. We can
        # ensure the vailidity of the data by removing the out of range data.

        filtered_distances = distances[
            (distances >= scan.range_min) & (distances <= scan.range_max) & np.isfinite(distances)
        ]

        # Ok, so I'm not sure how familiar you may or may not be with NumPy,
        # but look at the above magic! NumPy lets you index an array with a
        # boolean array. This may not seem special at first, but it allows
        # you to filter elements with the C code under the hood of NumPy
        # instead of using the slower Python `filter`.

        # Remember, however, that this is magic that NumPy provides, you
        # cannot do this on standard Python arrays.

        # Here's a small syntax breakdown on that:
        # distances[ ... ] <- We're indexing our array
        #
        #       ...[(distances >= scan.range_min) ...] <- We want to create an array with
        #                                                 `True` only in places where the
        #                                                 value >= `scan.range_min`.
        #
        #       ...[... (distances <= scan.range_max) ...] <- Same as above except creating an
        #                                                     array with `True` only where the
        #                                                     value <= `scan.range_max`.
        #
        #       ...[... np.isfinite(distances)] <- Similar to above, except the array only
        #                                          has `True` in locations where the value
        #                                          is not `NaN` or `Inf`.
        #
        #       ...[(...) & (...) & (...)] <- Take all of these boolean arrays and create a new
        #                                     array that only has `True` in locations where both
        #                                     arrays have `True` in them.
        #
        # distances[<boolean_array>] <- Create a 1 dimensional array (with caveats) of
        #                               values from locations where the `<boolean_array>`
        #                               has the value `True`.

        # Now, with our filtered_distances, we can quickly retrieve the smallest and largest
        # values to publish!

        min, max = filtered_distances.min(), filtered_distances.max()

        # Publish our data!
        self.pub_closest.publish(Float64(min))
        self.pub_farthest.publish(Float64(max))

    def run(self) -> None:
        """
        This method handles the main running loop for this node. For this case,
        it will simply be an infinite loop as all necessary functionality is
        handled by the subscriber callback.
        """
        ros.spin()

# Checks if this module is running as itself as opposed to being imported.
if __name__ == "__main__":
    ScanRanges().run()
