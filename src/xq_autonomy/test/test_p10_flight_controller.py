import numpy as np

from geometry_msgs.msg import Point
from traj_utils.msg import Bspline

from xq_autonomy.p10_flight_controller_node import P10FlightControllerNode


def _line() -> Bspline:
    message = Bspline()
    message.order = 1
    message.pos_pts = [Point(x=0.0), Point(x=2.0)]
    message.knots = [0.0, 0.0, 4.0, 4.0]
    return message


def test_trajectory_sampling_holds_endpoint_after_domain_end():
    position, feedforward = P10FlightControllerNode._trajectory_positions(_line(), 4.5)
    assert np.allclose(position, (2.0, 0.0, 0.0))
    assert np.allclose(feedforward, (0.0, 0.0, 0.0))


def test_trajectory_sampling_has_forward_motion_inside_domain():
    position, feedforward = P10FlightControllerNode._trajectory_positions(_line(), 2.0)
    assert position[0] == 1.0
    assert feedforward[0] > 0.0
