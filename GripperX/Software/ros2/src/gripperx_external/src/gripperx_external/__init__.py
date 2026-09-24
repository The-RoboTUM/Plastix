"""External-goal link between the Octopus overhead system and GripperX.

The modules in this package split into two groups:

* **pure** - ``octopus_protocol``, ``geodesy``, ``grasp``, ``validation``,
  ``arming``. These import no ``rclpy`` and no ROS message types. That is a
  hard constraint, not a preference: it is what makes the whole decision path
  testable with ``python3 test/check_*.py`` and nothing running, which matters
  because the real robot is blocked (dead EKF inputs) and the twin has no arm.
* **ROS-facing** - the rosbridge client, the link node, the goal gateway and
  the diagnostics assembly. Added in later build-order stages.
"""
